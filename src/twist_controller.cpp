#include <moveit_twist_controller/common.hpp>
#include <moveit_twist_controller/twist_controller.hpp>

#include <moveit/robot_state/conversions.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace moveit_twist_controller
{

MoveitTwistController::MoveitTwistController()
    : controller_interface::ControllerInterface(), initialized_( false ), enabled_( false ),
      reset_pose_( false ), reset_tool_center_( false ), move_tool_center_( false ),
      max_speed_gripper_( 0 ), free_angle_( -1 ), tool_center_offset_( Eigen::Affine3d::Identity() ),
      gripper_pos_( 0.0 ), gripper_speed_( 0.0 ), joint_state_received_( false ),
      gripper_upper_limit_( 0 ), gripper_lower_limit_( 0 ), hold_pose_( false )
{
}

controller_interface::InterfaceConfiguration MoveitTwistController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  if ( arm_joint_names_.empty() )
    RCLCPP_ERROR( get_node()->get_logger(),
                  "Arm joint names are empty in command_interface_configuration" );
  // all arm joints
  for ( const auto &joint_name : arm_joint_names_ ) {
    config.names.push_back( joint_name + "/position" );
  }
  // The gripper joint
  config.names.push_back( gripper_joint_name_ + std::string( "/position" ) );
  return config;
}

controller_interface::InterfaceConfiguration MoveitTwistController::state_interface_configuration() const
{
  // Get Position interfaces for all joints -> not just the ones we control
  // required for collision checking (e.g. arm with flipper)
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // request state for all joints
  for ( const auto &joint_name : joint_names_ ) {
    config.names.push_back( joint_name + "/position" );
    // config.names.push_back(joint_name + "/velocity");
  }

  return config;
}

controller_interface::CallbackReturn MoveitTwistController::on_init()
{
  try {

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>( get_node()->get_clock() );
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>( *tf_buffer_ );

    param_listener_ = std::make_shared<moveit_twist_controller::ParamListener>( get_node() );
  } catch ( const std::exception &e ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Exception during on_init: %s", e.what() );
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
MoveitTwistController::on_configure( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  try {
    auto params = param_listener_->get_params();
    free_angle_ = -1;
    if ( params.free_angle == "x" )
      free_angle_ = 0;
    else if ( params.free_angle == "y" )
      free_angle_ = 1;
    else if ( params.free_angle == "z" )
      free_angle_ = 2;

    // start status thread
    moveit_init_node_ =
        std::make_shared<rclcpp::Node>( "twist_controller_status", get_node()->get_namespace() );

    ik_.init( get_node(), moveit_init_node_, params.group_name );

    // Populate joint_names_ from the IK solver
    joint_names_ = ik_.getAllJointNames();
    arm_joint_names_ = ik_.getGroupJointNames();
    // extract the arm joint indices
    arm_joint_indices_.resize( arm_joint_names_.size() );
    for ( size_t i = 0; i < arm_joint_names_.size(); ++i ) {
      auto it = std::find( joint_names_.begin(), joint_names_.end(), arm_joint_names_[i] );
      if ( it != joint_names_.end() ) {
        arm_joint_indices_[i] = static_cast<int>( std::distance( joint_names_.begin(), it ) );
      } else {
        RCLCPP_ERROR( get_node()->get_logger(), "Joint %s not found in joint_names_",
                      arm_joint_names_[i].c_str() );
        return controller_interface::CallbackReturn::ERROR;
      }
    }
    joint_velocity_limits_ = ik_.getJointVelocityLimits();
    // print joint names and arm joint names
    std::stringstream debug;
    debug << "All joints:" << std::endl;
    for ( const auto &joint_name : joint_names_ ) { debug << joint_name << std::endl; }
    debug << "Arm joints:" << std::endl;
    for ( const auto &joint_name : arm_joint_names_ ) { debug << joint_name << std::endl; }
    debug << "Arm joint Velocity limits:" << std::endl;
    for ( size_t i = 0; i < arm_joint_names_.size(); ++i ) {
      debug << arm_joint_names_[i] << ": " << joint_velocity_limits_[i] << std::endl;
    }
    RCLCPP_INFO( get_node()->get_logger(), "%s", debug.str().c_str() );
    goal_state_.resize( arm_joint_names_.size() );
    current_joint_angles_.resize( joint_names_.size() );
    current_arm_joint_angles.resize( arm_joint_names_.size() );
    previous_goal_state_.resize( arm_joint_names_.size() );
    RCLCPP_INFO( get_node()->get_logger(), "goal_state_ size: %lu", goal_state_.size() );
    RCLCPP_INFO( get_node()->get_logger(), "previous_goal_state_ size: %lu",
                 previous_goal_state_.size() );

    gripper_joint_name_ = params.gripper_joint_name;
    RCLCPP_INFO( get_node()->get_logger(), "Gripper joint: %s", gripper_joint_name_.c_str() );
    if ( !loadGripperJointLimits() ) {
      return controller_interface::CallbackReturn::ERROR;
    }
    std::string teleop_ns = "teleop";
    // Create publishers and subscriptions.
    goal_pose_pub_ =
        get_node()->create_publisher<geometry_msgs::msg::PoseStamped>( teleop_ns + "/goal_pose", 10 );
    robot_state_pub_ =
        get_node()->create_publisher<moveit_msgs::msg::DisplayRobotState>( teleop_ns + "/robot_state", 10 );
    enabled_pub_ = get_node()->create_publisher<std_msgs::msg::Bool>( teleop_ns + "/enabled", 10 );

    // Twist command subscription
    twist_cmd_sub_ = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
        teleop_ns + "/eef_cmd", 10, [this]( const geometry_msgs::msg::TwistStamped::SharedPtr twist_msg ) {
          twist_.linear.x() = twist_msg->twist.linear.x; // TODO: store frame
          twist_.linear.y() = twist_msg->twist.linear.y;
          twist_.linear.z() = twist_msg->twist.linear.z;
          twist_.angular.x() = twist_msg->twist.angular.x;
          twist_.angular.y() = twist_msg->twist.angular.y;
          twist_.angular.z() = twist_msg->twist.angular.z;
        } );

    // Gripper speed subscription
    gripper_cmd_sub_ = get_node()->create_subscription<std_msgs::msg::Float64>(
        teleop_ns + "/gripper_cmd", 10, [this]( const std_msgs::msg::Float64::SharedPtr msg ) {
          gripper_speed_ = msg->data;
          ;
        } );

    // Services
    reset_pose_server_ = get_node()->create_service<std_srvs::srv::Empty>(
        teleop_ns + "/reset_pose", [this]( const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                              std::shared_ptr<std_srvs::srv::Empty::Response> response ) {
          (void)request;
          (void)response;
          reset_pose_ = true;
          return true;
        } );

    reset_tool_center_server_ = get_node()->create_service<std_srvs::srv::Empty>(
        teleop_ns + "/reset_tool_center", [this]( const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                     std::shared_ptr<std_srvs::srv::Empty::Response> response ) {
          (void)request;
          (void)response;
          reset_tool_center_ = true;
          return true;
        } );
    hold_pose_server_ = get_node()->create_service<std_srvs::srv::SetBool>(
        teleop_ns + "/hold_mode", [this]( const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                             std::shared_ptr<std_srvs::srv::SetBool::Response> response ) {
          if ( hold_pose_ != request->data ) {
            hold_pose_ = request->data;
            RCLCPP_INFO_STREAM( get_node()->get_logger(),
                                "Hold pose " << ( hold_pose_ ? "enabled" : "disabled" ) );
            if ( hold_pose_ ) {
              hold_goal_pose_ = getPoseInFrame( ee_goal_pose_, "odom" );
            }
          }
          response->success = true;
          return true;
        } );

    move_tool_center_server_ = get_node()->create_service<std_srvs::srv::SetBool>(
        teleop_ns + "/move_tool_center", [this]( const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                    std::shared_ptr<std_srvs::srv::SetBool::Response> response ) {
          move_tool_center_ = request->data;
          response->success = true;
          return true;
        } );
    // TEST if tf is remapped correctly
    // test whether the frame base_link or the frame athene/base_link is available
    tf2::Stamped<tf2::Transform> transform;
    try {
      bool base_link_exists = tf_buffer_->_frameExists("base_link" );
      bool athene_base_link_exists = tf_buffer_->_frameExists("athena/base_link" );
      if ( base_link_exists ) {
        RCLCPP_WARN( get_node()->get_logger(), "base_link exists" );
      } else {
        RCLCPP_WARN( get_node()->get_logger(), "base_link does not exist" );
      }
      if ( athene_base_link_exists ) {
          RCLCPP_WARN( get_node()->get_logger(), "athena/base_link exists" );
      } else {
          RCLCPP_WARN( get_node()->get_logger(), "athena/base_link does not exist" );
      }
    } catch ( tf2::TransformException &ex ) {
      RCLCPP_ERROR( get_node()->get_logger(), "Error: %s", ex.what() );
    }
    RCLCPP_WARN( get_node()->get_logger(), "Namespace: %s", get_node()->get_namespace() );
  } catch ( const std::exception &e ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Exception during on_configure: %s", e.what() );
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
MoveitTwistController::on_activate( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  if ( enabled_ )
    return controller_interface::CallbackReturn::SUCCESS;

  // Initialize states
  twist_.linear = Eigen::Vector3d::Zero();
  twist_.angular = Eigen::Vector3d::Zero();
  reset_pose_ = true;
  reset_tool_center_ = false;
  move_tool_center_ = false;
  hold_pose_ = false;

  if ( auto handle_it = std::find_if( state_interfaces_.begin(), state_interfaces_.end(),
                                      [this]( const auto &iface ) {
                                        return iface.get_name() ==
                                               gripper_joint_name_ + "/position";
                                      } );
       handle_it != state_interfaces_.end() ) {
    gripper_pos_ = handle_it->get_value();
  } else {
    gripper_pos_ = 0.0;
  }
  gripper_speed_ = 0.0;

  initialized_ = true;
  enabled_ = true;

  // activate publishers
  goal_pose_pub_->on_activate();
  robot_state_pub_->on_activate();
  enabled_pub_->on_activate();
  RCLCPP_INFO( get_node()->get_logger(), "Joystick Control started." );
  publishStatus();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
MoveitTwistController::on_deactivate( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  if ( !enabled_ )
    return controller_interface::CallbackReturn::SUCCESS;

  RCLCPP_INFO( get_node()->get_logger(), "Joystick Control stopped." );
  enabled_ = false;
  publishStatus();
  hideRobotState();

  // deactivate publishers
  goal_pose_pub_->on_deactivate();
  robot_state_pub_->on_deactivate();
  enabled_pub_->on_deactivate();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MoveitTwistController::update( const rclcpp::Time &time,
                                                                 const rclcpp::Duration &period )
{
  if ( !initialized_ ) {
    return controller_interface::return_type::OK;
  }

  // Retrieve current joint states from the hardware (read from state_interfaces_)
  for ( size_t i = 0; i < joint_names_.size(); ++i ) {
    const auto &joint_name = joint_names_[i];
    auto it = std::find_if( state_interfaces_.begin(), state_interfaces_.end(),
                            [&joint_name]( const auto &iface ) {
                              return iface.get_name() == joint_name + "/position";
                            } );
    if ( it != state_interfaces_.end() ) {
      current_joint_angles_[i] = it->get_value();
    }
  }
  // extract arm joint angles
  for ( size_t i = 0; i < arm_joint_indices_.size(); ++i ) {
    current_arm_joint_angles[i] = current_joint_angles_[arm_joint_indices_[i]];
  }

  // Compute next state
  updateArm( time, period );
  updateGripper( time, period );

  // Visualization // TODO: make realtime safe
  publishRobotState( goal_state_, contact_map_ );

  // Publish goal pose
  geometry_msgs::msg::PoseStamped goal_pose_msg;
  goal_pose_msg.header.frame_id = ik_.getBaseFrame();
  goal_pose_msg.pose = tf2::toMsg( tool_goal_pose_ );
  goal_pose_msg.header.stamp = time;
  goal_pose_pub_->publish( goal_pose_msg );

  return controller_interface::return_type::OK;
}

double MoveitTwistController::computeJointAngleDiff( const double angle_1,
                                                        const double angle_2 ) const
{
  // First compute the naive difference
  double diff = angle_2 - angle_1;

  // Normalize the difference into the range [-pi, pi]
  diff = fmod(diff, 2.0 * M_PI);
  if (diff > M_PI) {
    diff -= 2.0 * M_PI;
  } else if (diff < -M_PI) {
    diff += 2.0 * M_PI;        // shift from [-2*pi, -pi) to [0, pi)
  }

  return diff;
}

void MoveitTwistController::updateArm( const rclcpp::Time & /*time*/, const rclcpp::Duration &period )
{
  Eigen::Affine3d old_goal = ee_goal_pose_;
  previous_goal_state_ = goal_state_;

  // Compute new goal pose
  computeNewGoalPose( period );

  // Compute IK
  if ( ik_.calcInvKin( ee_goal_pose_, previous_goal_state_, goal_state_ ) ) {
    contact_map_.clear();
    sensor_msgs::msg::JointState joint_state;
    joint_state.name = joint_names_;
    joint_state.position = current_joint_angles_;
    bool collision_free = ik_.isCollisionFree( joint_state, goal_state_, contact_map_ );
    if ( !collision_free ) {
      ee_goal_pose_ = old_goal;
      goal_state_ = previous_goal_state_;
      RCLCPP_WARN( get_node()->get_logger(), "Collision detected." );
    }
  } else {
    // IK failed
    ee_goal_pose_ = old_goal;
    goal_state_ = previous_goal_state_;
    RCLCPP_WARN( get_node()->get_logger(), "IK failed." );
  }
  // Make sure that the change in joint angles is not too large -> velocity limits
  double max_velocity_factor = 0;
  for ( size_t i=0; i<goal_state_.size(); ++i ) {
    const double angle_diff = std::abs(computeJointAngleDiff(  previous_goal_state_[i], goal_state_[i] ));
    max_velocity_factor = std::max( max_velocity_factor, angle_diff/(joint_velocity_limits_[i]*period.seconds()) );
    if (angle_diff/(joint_velocity_limits_[i]*period.seconds())>1.0) {
      RCLCPP_WARN( get_node()->get_logger(), "Joint %s: Max change in joint angles is > 1.0 Old State: %f, New State: %f, angle diff %f", arm_joint_names_[i].c_str(), previous_goal_state_[i], goal_state_[i], angle_diff );
    }
  }

  if (max_velocity_factor>1.0) {
    RCLCPP_WARN( get_node()->get_logger(), "Max velocity factor: %f. Rejected goal state.", max_velocity_factor );
    reset_pose_ = true;
    goal_state_ = previous_goal_state_;
  }else {
    // avoid jumps in joint angles especially in the continuous joints!!
    for ( size_t i=0; i<goal_state_.size(); ++i ) {
      goal_state_[i] = previous_goal_state_[i] + computeJointAngleDiff( previous_goal_state_[i], goal_state_[i] );
    }
  }


  bool success = true;
  // Write next goal state to command_interfaces_
  for ( size_t i = 0; i < arm_joint_names_.size(); ++i ) {
    const auto &joint_name = arm_joint_names_[i];
    auto it = std::find_if( command_interfaces_.begin(), command_interfaces_.end(),
                            [&joint_name]( const auto &iface ) {
                              return iface.get_name() == joint_name + "/position";
                            } );
    if ( it != command_interfaces_.end() )
      success &= it->set_value( goal_state_[i] );
    else
      success = false;
  }
  if ( !success )
    RCLCPP_WARN( get_node()->get_logger(), "Failed to write next goal state to hardware." );
}

bool MoveitTwistController::computeNewGoalPose( const rclcpp::Duration &period )
{
  if ( reset_pose_ ) {
    // Reset end-effector goal
    previous_goal_state_ = current_arm_joint_angles;
    ee_goal_pose_ = ik_.getEndEffectorPose( current_arm_joint_angles );
    tool_goal_pose_ = ee_goal_pose_ * tool_center_offset_;
    reset_pose_ = false;
    return true;
  }

  if ( reset_tool_center_ ) {
    // Reset tool center to end-effector goal
    tool_center_offset_ = Eigen::Affine3d::Identity();
    tool_goal_pose_ = ee_goal_pose_;
    reset_tool_center_ = false;
    return false;
  }

  if ( hold_pose_ ) {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_->lookupTransform(
          ik_.getBaseFrame(), hold_goal_pose_.header.frame_id, tf2::TimePointZero );
    } catch ( tf2::TransformException &ex ) {
      RCLCPP_WARN( get_node()->get_logger(), "%s", ex.what() );
      return false;
    }
    Eigen::Affine3d base_to_frame = tf2::transformToEigen( transform_stamped.transform );
    Eigen::Affine3d frame_to_ee;
    tf2::fromMsg( hold_goal_pose_.pose, frame_to_ee );
    ee_goal_pose_ = base_to_frame * frame_to_ee;
    tool_goal_pose_ = ee_goal_pose_ * tool_center_offset_;
    return true;
  }

  // Update end-effector pose with current command (twist_)
  if ( twist_.linear.isZero() && twist_.angular.isZero() ) {
    return false;
  }

  const auto rot = rpyToRot( period.seconds() * twist_.angular );
  Eigen::Affine3d twist_transform( rot );
  twist_transform.translation() = period.seconds() * twist_.linear;

  const Eigen::Affine3d tool_center_movement = tool_center_offset_ * twist_transform;
  tool_goal_pose_ = ee_goal_pose_ * tool_center_movement;

  if ( !move_tool_center_ ) {
    // Move end-effector
    ee_goal_pose_ = tool_goal_pose_ * tool_center_offset_.inverse( Eigen::Isometry );
    return true;
  } else {
    // Move tool frame
    tool_center_offset_ = tool_center_movement;
    return false;
  }
}

void MoveitTwistController::updateGripper( const rclcpp::Time & /*time*/,
                                           const rclcpp::Duration &period )
{
  gripper_pos_ += period.seconds() * gripper_speed_;
  gripper_pos_ = std::min( gripper_upper_limit_, std::max( gripper_lower_limit_, gripper_pos_ ) );
  bool success = false;
  // Write gripper command
  auto it = std::find_if( command_interfaces_.begin(), command_interfaces_.end(),
                          [this]( const auto &iface ) {
                            return iface.get_name() == gripper_joint_name_ + "/position";
                          } );
  if ( it != command_interfaces_.end() ) {
    success = it->set_value( gripper_pos_ );
  }
  if ( !success ) {
    RCLCPP_WARN( get_node()->get_logger(), "Failed to write gripper command to hardware." );
  }
}

bool MoveitTwistController::loadGripperJointLimits()
{
  if ( !ik_.getJointLimits( gripper_joint_name_, gripper_lower_limit_, gripper_upper_limit_ ) ) {
    RCLCPP_WARN( get_node()->get_logger(), "Failed to load gripper joint limits." );
    return false;
  }
  RCLCPP_INFO( get_node()->get_logger(), "Gripper joint limits: %f %f", gripper_lower_limit_,
               gripper_upper_limit_ );
  return true;
}

void MoveitTwistController::publishRobotState(
    const std::vector<double> &arm_joint_states,
    const collision_detection::CollisionResult::ContactMap &contact_map )
{
  sensor_msgs::msg::JointState joint_state;
  joint_state.name = joint_names_;
  joint_state.position = current_joint_angles_;
  moveit::core::RobotState robot_state =
      ik_.getAsRobotState( joint_state, arm_joint_states );

  // transform the base
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_->lookupTransform( "world", ik_.getBaseFrame(), tf2::TimePointZero );
  } catch ( tf2::TransformException &ex ) {
    RCLCPP_WARN( get_node()->get_logger(), "%s", ex.what() );
    return;
  }

  Eigen::Affine3d pose = tf2::transformToEigen( transform_stamped.transform );
  updateRobotStatePose( robot_state, pose );

  moveit_msgs::msg::DisplayRobotState display_robot_state;
  moveit::core::robotStateToRobotStateMsg( robot_state, display_robot_state.state );

  // highlight links in collision
  std_msgs::msg::ColorRGBA color;
  color.a = 1.0;
  color.r = 1.0;
  color.g = 0.0;
  color.b = 0.0;

  for ( const auto &it : contact_map ) {
    const std::string &link1 = it.first.first;
    const std::string &link2 = it.first.second;

    moveit_msgs::msg::ObjectColor object_color;
    object_color.id = link1;
    object_color.color = color;
    display_robot_state.highlight_links.push_back( object_color );

    object_color.id = link2;
    object_color.color = color;
    display_robot_state.highlight_links.push_back( object_color );
  }

  robot_state_pub_->publish( display_robot_state );
}

geometry_msgs::msg::PoseStamped MoveitTwistController::getPoseInFrame( const Eigen::Affine3d &pose,
                                                                       const std::string &frame )
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = frame;

  try {
    transform_stamped = tf_buffer_->lookupTransform( frame, ik_.getBaseFrame(), tf2::TimePointZero );
  } catch ( tf2::TransformException &ex ) {
    RCLCPP_WARN( get_node()->get_logger(), "%s", ex.what() );
    // Return pose in the base frame if transform fails.
    pose_stamped.pose = tf2::toMsg( pose );
    pose_stamped.header.stamp = get_node()->now();
    return pose_stamped;
  }

  Eigen::Affine3d frame_to_base = tf2::transformToEigen( transform_stamped.transform );
  Eigen::Affine3d frame_to_pose = frame_to_base * pose;

  pose_stamped.header.stamp = transform_stamped.header.stamp;
  pose_stamped.pose = tf2::toMsg( frame_to_pose );
  return pose_stamped;
}

void MoveitTwistController::publishStatus() const
{
  std_msgs::msg::Bool bool_msg;
  bool_msg.data = enabled_;
  enabled_pub_->publish( bool_msg );
}

void MoveitTwistController::hideRobotState() const
{
  moveit_msgs::msg::DisplayRobotState display_robot_state;
  display_robot_state.hide = true;
  robot_state_pub_->publish( display_robot_state );
}

} // namespace moveit_twist_controller

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS( moveit_twist_controller::MoveitTwistController,
                        controller_interface::ControllerInterface )