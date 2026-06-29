#include <moveit_twist_controller/common.hpp>
#include <moveit_twist_controller/twist_controller.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace moveit_twist_controller
{

MoveitTwistController::MoveitTwistController() : tool_center_offset_( Eigen::Affine3d::Identity() )
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
  const std::string prefix =
      params_.chained_controller.empty() ? "" : params_.chained_controller + "/";
  size_t index = 0;
  for ( const auto &joint_name : arm_joint_names_ ) {
    config.names.push_back( prefix + joint_name + "/position" );
    joint_command_interface_mapping_[joint_name + "/position"] = index++;
  }
  return config;
}

controller_interface::InterfaceConfiguration MoveitTwistController::state_interface_configuration() const
{
  // Get Position interfaces for all joints -> not just the ones we control
  // required for collision checking (e.g. arm with flipper)
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // request state for all joints
  for ( size_t i = 0; i < joint_names_.size(); i++ ) {
    const auto &joint_name = joint_names_[i];
    config.names.push_back( joint_name + "/position" );
    joint_state_interface_mapping_[joint_name + "/position"] = i;
  }

  return config;
}

controller_interface::CallbackReturn MoveitTwistController::on_init()
{
  try {

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(
        get_node()->get_clock(), tf2::BUFFER_CORE_DEFAULT_CACHE_TIME, get_node() );
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>( *tf_buffer_, get_node() );

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
    params_ = param_listener_->get_params();

    if ( params_.free_angle == "x" )
      free_angle_ = 0;
    else if ( params_.free_angle == "y" )
      free_angle_ = 1;
    else if ( params_.free_angle == "z" )
      free_angle_ = 2;

    // create a node to initialize the IK solver
    moveit_init_node_ = std::make_shared<rclcpp::Node>(
        get_node()->get_name() + std::string( "_moveit_init" ), get_node()->get_namespace() );

    if ( !ik_.init( get_node(), moveit_init_node_, params_.group_name, get_robot_description(),
                    params_.robot_descriptions_loading_timeout ) )
      return controller_interface::CallbackReturn::ERROR;

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
    joint_velocity_limits_ = params_.velocity_limits;
    if ( const auto srdf_limits = ik_.getJointVelocityLimits();
         srdf_limits.size() != joint_velocity_limits_.size() ) {
      RCLCPP_WARN( get_node()->get_logger(),
                   "The given velocity limits (%lu) do not match the "
                   "velocity limits from the SRDF (%lu). Using the SRDF limits.",
                   joint_velocity_limits_.size(), srdf_limits.size() );
      joint_velocity_limits_ = srdf_limits;
    }
    goal_state_.resize( arm_joint_names_.size() );
    current_joint_angles_.resize( joint_names_.size() );
    current_arm_joint_angles.resize( arm_joint_names_.size() );
    previous_goal_state_.resize( arm_joint_names_.size() );
    smoothed_joint_velocities_.assign( arm_joint_names_.size(), 0.0 );

    // Create publishers and subscriptions.
    goal_pose_pub_ =
        get_node()->create_publisher<geometry_msgs::msg::PoseStamped>( "~/goal_pose", 10 );
    collision_marker_pub_ = get_node()->create_publisher<visualization_msgs::msg::MarkerArray>(
        "~/collision_markers", 10 );
    rt_collision_marker_pub_ =
        std::make_unique<realtime_tools::RealtimePublisher<visualization_msgs::msg::MarkerArray>>(
            collision_marker_pub_ );
    enabled_pub_ = get_node()->create_publisher<std_msgs::msg::Bool>( "~/enabled", 10 );

    // Twist command subscription
    twist_cmd_sub_ = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
        "~/eef_cmd", 10, [this]( const geometry_msgs::msg::TwistStamped::SharedPtr twist_msg ) {
          tf2::Quaternion tf2_quat;
          if ( twist_msg->header.frame_id.empty() ) {
            tf2_quat = tf2::Quaternion::getIdentity();
          } else {
            // Transform the twist into the eef tip frame
            geometry_msgs::msg::TransformStamped transform_stamped;
            try {
              transform_stamped = tf_buffer_->lookupTransform(
                  ik_.getTipFrame(), twist_msg->header.frame_id, tf2::TimePointZero );
            } catch ( tf2::TransformException &ex ) {
              RCLCPP_WARN( get_node()->get_logger(), "%s", ex.what() );
              return;
            }
            tf2_quat = tf2::Quaternion(
                transform_stamped.transform.rotation.x, transform_stamped.transform.rotation.y,
                transform_stamped.transform.rotation.z, transform_stamped.transform.rotation.w );
          }

          const tf2::Matrix3x3 rotation( tf2_quat );

          const tf2::Vector3 lin_in( twist_msg->twist.linear.x, twist_msg->twist.linear.y,
                                     twist_msg->twist.linear.z );
          const tf2::Vector3 lin_out = rotation * lin_in;

          const tf2::Vector3 ang_in( twist_msg->twist.angular.x, twist_msg->twist.angular.y,
                                     twist_msg->twist.angular.z );
          const tf2::Vector3 ang_out = rotation * ang_in;

          TwistCommand cmd;
          cmd.twist.linear = Eigen::Vector3d( lin_out.x(), lin_out.y(), lin_out.z() );
          cmd.twist.angular = Eigen::Vector3d( ang_out.x(), ang_out.y(), ang_out.z() );
          cmd.stamp = get_node()->now();
          twist_cmd_buf_.writeFromNonRT( cmd );
        } );

    // Services
    reset_pose_server_ = get_node()->create_service<std_srvs::srv::Empty>(
        "~/reset_pose", [this]( const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                std::shared_ptr<std_srvs::srv::Empty::Response> response ) {
          (void)request;
          (void)response;
          reset_pose_ = true;
          return true;
        } );

    reset_tool_center_server_ = get_node()->create_service<std_srvs::srv::Empty>(
        "~/reset_tool_center", [this]( const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                       std::shared_ptr<std_srvs::srv::Empty::Response> response ) {
          (void)request;
          (void)response;
          reset_tool_center_ = true;
          return true;
        } );
    hold_pose_server_ = get_node()->create_service<std_srvs::srv::SetBool>(
        "~/hold_mode", [this]( const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                               std::shared_ptr<std_srvs::srv::SetBool::Response> response ) {
          if ( hold_pose_.load() != request->data ) {
            RCLCPP_INFO_STREAM( get_node()->get_logger(),
                                "Hold pose " << ( request->data ? "enabled" : "disabled" ) );
            if ( request->data ) {
              hold_goal_pose_buf_.writeFromNonRT( getPoseInFrame( ee_goal_pose_, "odom" ) );
            }
            hold_pose_.store( request->data );
          }
          response->success = true;
          return true;
        } );

    move_tool_center_server_ = get_node()->create_service<std_srvs::srv::SetBool>(
        "~/move_tool_center", [this]( const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                      std::shared_ptr<std_srvs::srv::SetBool::Response> response ) {
          move_tool_center_ = request->data;
          response->success = true;
          return true;
        } );
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
  const auto clock_type = get_node()->get_clock()->get_clock_type();
  TwistCommand zero_twist;
  zero_twist.stamp = rclcpp::Time( 0, 0, clock_type );
  twist_cmd_buf_.writeFromNonRT( zero_twist );
  reset_pose_.store( true );
  reset_tool_center_.store( false );
  move_tool_center_.store( false );
  hold_pose_.store( false );

  std::fill( smoothed_joint_velocities_.begin(), smoothed_joint_velocities_.end(), 0.0 );

  initialized_ = true;
  enabled_ = true;

  // activate publishers
  goal_pose_pub_->on_activate();
  collision_marker_pub_->on_activate();
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
  // deactivate publishers
  goal_pose_pub_->on_deactivate();
  collision_marker_pub_->on_deactivate();
  enabled_pub_->on_deactivate();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MoveitTwistController::update( const rclcpp::Time &time,
                                                                 const rclcpp::Duration &period )
{
  if ( !initialized_ ) {
    return controller_interface::return_type::OK;
  }

  // Refresh runtime-updatable parameters (non-blocking, RT-safe)
  param_listener_->try_update_params( params_ );

  // Retrieve current joint states from the hardware (read from state_interfaces_)
  for ( size_t i = 0; i < joint_names_.size(); ++i ) {
    if ( !getState( current_joint_angles_[i], joint_names_[i], "position" ) ) {
      RCLCPP_WARN( get_node()->get_logger(), "Failed to read joint state from hardware." );
      return controller_interface::return_type::ERROR;
    }
  }
  // extract arm joint angles
  for ( size_t i = 0; i < arm_joint_indices_.size(); ++i ) {
    current_arm_joint_angles[i] = current_joint_angles_[arm_joint_indices_[i]];
  }

  // Read commands from realtime buffers and apply timeout
  const double cmd_timeout = params_.cmd_timeout;
  {
    const auto &twist_cmd = *twist_cmd_buf_.readFromRT();
    if ( ( time - twist_cmd.stamp ).seconds() > cmd_timeout ) {
      twist_.linear = Eigen::Vector3d::Zero();
      twist_.angular = Eigen::Vector3d::Zero();
    } else {
      twist_ = twist_cmd.twist;
    }
  }

  // Compute next state
  updateArm( time, period );

  publishCollisionMarkers();

  // Publish goal pose
  geometry_msgs::msg::PoseStamped goal_pose_msg;
  goal_pose_msg.header.frame_id = ik_.getBaseFrame();
  goal_pose_msg.pose = tf2::toMsg( tool_goal_pose_ );
  goal_pose_msg.header.stamp = time;
  goal_pose_pub_->publish( goal_pose_msg );

  return controller_interface::return_type::OK;
}

double MoveitTwistController::computeJointAngleDiff( const double angle_1, const double angle_2 )
{
  // First compute the naive difference
  double diff = angle_2 - angle_1;

  // Normalize the difference into the range [-pi, pi]
  diff = fmod( diff, 2.0 * M_PI );
  if ( diff > M_PI ) {
    diff -= 2.0 * M_PI;
  } else if ( diff < -M_PI ) {
    diff += 2.0 * M_PI; // shift from [-2*pi, -pi) to [0, pi)
  }

  return diff;
}

bool MoveitTwistController::calculateInverseKinematicsConsideringVelocityLimits(
    Eigen::Affine3d &new_eef_pose, const Eigen::Affine3d &old_eef_pose,
    const rclcpp::Duration &period )
{
  double factor = 1.0;
  int count = 0;

  while ( count++ < params_.velocity_limit_satisfaction_max_iterations ) {
    // Try IK
    if ( ik_.calcInvKin( new_eef_pose, previous_goal_state_, goal_state_ ) ) {
      double max_velocity_factor = 0;
      for ( size_t i = 0; i < goal_state_.size(); ++i ) {
        const double angle_diff =
            std::abs( computeJointAngleDiff( previous_goal_state_[i], goal_state_[i] ) );
        max_velocity_factor = std::max(
            max_velocity_factor, angle_diff / ( joint_velocity_limits_[i] * period.seconds() ) );
      }
      if ( max_velocity_factor <= 1.0 ) {
        return true;
      }
    }

    factor *= params_.velocity_limit_satisfaction_multiplicator;

    // Interpolate translation
    const Eigen::Vector3d interp_translation =
        old_eef_pose.translation() +
        factor * ( new_eef_pose.translation() - old_eef_pose.translation() );

    // Interpolate rotation using slerp
    Eigen::Quaterniond old_q( old_eef_pose.rotation() );
    Eigen::Quaterniond new_q( new_eef_pose.rotation() );
    Eigen::Quaterniond interp_q = old_q.slerp( factor, new_q );

    // Create interpolated pose
    new_eef_pose.linear() = interp_q.toRotationMatrix();
    new_eef_pose.translation() = interp_translation;
  }
  return false;
}

void MoveitTwistController::updateArm( const rclcpp::Time & /*time*/, const rclcpp::Duration &period )
{
  const Eigen::Affine3d old_goal = ee_goal_pose_;
  previous_goal_state_ = goal_state_;

  // Compute new goal pose
  computeNewGoalPose( period );

  // Compute IK
  if ( calculateInverseKinematicsConsideringVelocityLimits( ee_goal_pose_, old_goal, period ) ) {
    // Apply joint velocity smoothing
    const double alpha = params_.joint_velocity_smoothing_factor;
    if ( alpha > 0.0 ) {
      const double dt = period.seconds();
      for ( size_t i = 0; i < goal_state_.size(); ++i ) {
        const double raw_vel = computeJointAngleDiff( previous_goal_state_[i], goal_state_[i] ) / dt;
        smoothed_joint_velocities_[i] =
            alpha * smoothed_joint_velocities_[i] + ( 1.0 - alpha ) * raw_vel;
        goal_state_[i] = previous_goal_state_[i] + smoothed_joint_velocities_[i] * dt;
      }
      ee_goal_pose_ = ik_.getEndEffectorPose( goal_state_ );
      tool_goal_pose_ = ee_goal_pose_ * tool_center_offset_;
    }

    contact_map_.clear();
    sensor_msgs::msg::JointState joint_state;
    joint_state.name = joint_names_;
    joint_state.position = current_joint_angles_;
    if ( !ik_.isCollisionFree( joint_state, goal_state_, contact_map_ ) ) {
      ee_goal_pose_ = old_goal;
      goal_state_ = previous_goal_state_;
      RCLCPP_DEBUG( get_node()->get_logger(), "Collision detected." );
    }
  } else {
    // IK failed
    ee_goal_pose_ = ik_.getEndEffectorPose( previous_goal_state_ );
    goal_state_ = previous_goal_state_;
    RCLCPP_DEBUG( get_node()->get_logger(), "IK failed." );
  }

  bool success = true;
  // Write next goal state to command_interfaces_
  for ( size_t i = 0; i < arm_joint_names_.size(); ++i ) {
    success &= setCommand( goal_state_[i], arm_joint_names_[i], "position" );
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

  if ( hold_pose_.load() ) {
    const auto hold_goal_pose = *hold_goal_pose_buf_.readFromRT();
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_->lookupTransform(
          ik_.getBaseFrame(), hold_goal_pose.header.frame_id, tf2::TimePointZero );
    } catch ( tf2::TransformException &ex ) {
      RCLCPP_WARN( get_node()->get_logger(), "%s", ex.what() );
      return false;
    }
    Eigen::Affine3d base_to_frame = tf2::transformToEigen( transform_stamped.transform );
    Eigen::Affine3d frame_to_ee;
    tf2::fromMsg( hold_goal_pose.pose, frame_to_ee );
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

bool MoveitTwistController::setCommand( const double value, const std::string &joint_name,
                                        const std::string &interface_name )
{
  return command_interfaces_[joint_command_interface_mapping_[joint_name + "/" + interface_name]]
      .set_value( value );
}

bool MoveitTwistController::getState( double &value, const std::string &joint_name,
                                      const std::string &interface_name ) const
{
  const auto state =
      state_interfaces_[joint_state_interface_mapping_[joint_name + "/" + interface_name]].get_optional();
  value = state.value();
  return state.has_value();
}

void MoveitTwistController::publishCollisionMarkers()
{
  if ( !rt_collision_marker_pub_->trylock() )
    return;

  auto &marker_array = rt_collision_marker_pub_->msg_;
  marker_array.markers.clear();

  // Always publish a DELETEALL marker first to clear stale markers
  visualization_msgs::msg::Marker delete_all;
  delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
  delete_all.header.frame_id = ik_.getBaseFrame();
  marker_array.markers.push_back( delete_all );

  int id = 0;
  for ( const auto &[pair, contacts] : contact_map_ ) {
    for ( const auto &contact : contacts ) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = ik_.getBaseFrame();
      marker.ns = "collisions";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = contact.pos.x();
      marker.pose.position.y = contact.pos.y();
      marker.pose.position.z = contact.pos.z();
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.03;
      marker.scale.y = 0.03;
      marker.scale.z = 0.03;
      marker.color.r = 1.0f;
      marker.color.a = 1.0f;
      marker_array.markers.push_back( marker );
    }
  }

  rt_collision_marker_pub_->unlockAndPublish();
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

  const Eigen::Affine3d frame_to_base = tf2::transformToEigen( transform_stamped.transform );
  const Eigen::Affine3d frame_to_pose = frame_to_base * pose;

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

} // namespace moveit_twist_controller

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS( moveit_twist_controller::MoveitTwistController,
                        controller_interface::ControllerInterface )