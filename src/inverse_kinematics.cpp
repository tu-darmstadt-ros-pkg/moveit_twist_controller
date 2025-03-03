#include <math.h>
#include <memory>
#include <moveit_twist_controller/inverse_kinematics.hpp>
#include <oneapi/tbb/partitioner.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace moveit_twist_controller
{
bool InverseKinematics::init( rclcpp::Node::SharedPtr node, const std::string &group_name )
{
  node_ = node;
  RCLCPP_INFO( node_->get_logger(), "Initializing inverse kinematics controller in namespace '%s'.",
               node_->get_namespace() );
  RCLCPP_INFO( node_->get_logger(), "Group name: %s", group_name.c_str() );
  auto robot_description = getParameterFromTopic( "robot_description" );
  auto robot_description_semantic = getParameterFromTopic( "robot_description_semantic" );
  if ( robot_description.empty() || robot_description_semantic.empty() ) {
    RCLCPP_ERROR( node_->get_logger(), "Failed to get robot description." );
    return false;
  }
  setKinematicParameters();
  robot_model_loader::RobotModelLoader::Options opt;
  opt.urdf_string_ = robot_description;
  opt.srdf_string = robot_description_semantic;
  opt.robot_description = "robot_description";

  robot_model_loader_.reset( new robot_model_loader::RobotModelLoader( node_, opt ) );
  robot_model_ = robot_model_loader_->getModel();
  planning_scene_ = std::make_shared<planning_scene::PlanningScene>( robot_model_ );
  robot_state_.reset( new moveit::core::RobotState( robot_model_ ) );
  robot_state_->setToDefaultValues();

  // load joint model group
  joint_model_group_ = robot_state_->getJointModelGroup( group_name );
  if ( joint_model_group_ == NULL ) {
    RCLCPP_WARN_STREAM( node_->get_logger(),
                        "Joint model group '" << group_name << "' does not exist." );
    return false;
  }
  arm_joint_names_ = joint_model_group_->getActiveJointModelNames();
  joint_names_ = robot_model_->getActiveJointModelNames();
  // remove world_virtual_joint
  if (joint_names_.size()>0 && joint_names_[0] == "world_virtual_joint")
    joint_names_.erase(joint_names_.begin());
  // Retrieve solver
  const kinematics::KinematicsBaseConstPtr &solver = joint_model_group_->getSolverInstance();
  if ( !solver ) {
    RCLCPP_ERROR( node_->get_logger(),
                  "No IK solver loaded for group %s, cannot set group configuration via IK.",
                  joint_model_group_->getName().c_str() );
    return false;
  }
  tip_frame_ = solver->getTipFrame();

  // Debug output
  std::stringstream debug;
  debug << "Initialized inverse kinematics controller with joint group '" << group_name << "'. "
        << std::endl;
  debug << "Tip frame: " << tip_frame_ << std::endl;
  debug << "Base frame: " << solver->getBaseFrame() << std::endl;
  debug << "Joints:" << std::endl;
  for ( unsigned int i = 0; i < arm_joint_names_.size(); i++ ) {
    debug << i << ": " << arm_joint_names_[i] << std::endl;
  }
  RCLCPP_INFO_STREAM( node_->get_logger(), debug.str() );

  return true;
}

bool InverseKinematics::getJointLimits( const std::string &joint_name, double &lower,
                                        double &upper ) const
{
  if ( robot_model_ ) {
    if ( const auto joint_model = robot_model_->getJointModel( joint_name ) ) {
      const auto bounds = joint_model->getVariableBounds( joint_model->getVariableNames()[0] );
      lower = bounds.min_position_;
      upper = bounds.max_position_;
      return true;
    }
  }
  return false;
}

bool InverseKinematics::calcInvKin( const Eigen::Affine3d &pose, const std::vector<double> &seed,
                                    std::vector<double> &solution )
{
  if ( seed.size() != arm_joint_names_.size() ) {
    RCLCPP_INFO(
        node_->get_logger(),
        "[InverseKinematics::calcInvKin] Seed size (%lu) does not match number of joints (%lu)",
        seed.size(), arm_joint_names_.size() );
    return false;
  }
  geometry_msgs::msg::Pose pose_msg = tf2::toMsg( pose );

  moveit_msgs::msg::MoveItErrorCodes error_code;
  solution.resize( arm_joint_names_.size() );
  if ( !joint_model_group_->getSolverInstance()->searchPositionIK( pose_msg, seed, 0.01, solution,
                                                                   error_code ) ) {
    RCLCPP_WARN_STREAM( node_->get_logger(),
                        "Computing IK from "
                            << joint_model_group_->getSolverInstance()->getBaseFrame() << " to "
                            << joint_model_group_->getSolverInstance()->getTipFrame()
                            << " failed. Error code: " << error_code.val << " ("
                            << moveitErrCodeToString( error_code.val ) << ")" );
    return false;
  }

  //  double limit = 10 * M_PI / 180;

  //  // find maximum position change
  //  double min_change_factor = 1;
  //  double requested_change = 0;
  //  for (unsigned int i = 0; i < solution.size(); i++) {
  //      double change = std::abs(seed[i] - solution[i]);
  //      if (change != 0.0) {
  //          double factor = limit/change;
  //          if (factor < min_change_factor) {
  //              min_change_factor = factor;
  //              requested_change = change;
  //          }
  //      }
  //  }

  //  // limit joint angle change
  //  if (min_change_factor < 1) {
  //      for (unsigned int i = 0; i < solution.size(); i++) {
  //          double change = solution[i] - seed[i];
  //          solution[i] =  seed[i] + (change * min_change_factor);
  //      }
  //      ROS_WARN_STREAM_THROTTLE(1, "Joint angle change (" << requested_change << ") bigger than
  //      max (" << limit << "). Limiting speed with factor: " << min_change_factor << ".");
  //      std::stringstream debug; debug << "Current\t | \t Requested" << std::endl; for (unsigned
  //      int i = 0; i < solution.size(); i++) {
  //          debug << seed[i] << "\t\t" << solution[i] << std::endl;
  //      }
  //      ROS_WARN_STREAM_THROTTLE(1, debug.str());
  //  }

  return true;
}

Eigen::Affine3d InverseKinematics::getEndEffectorPose( const std::vector<double> &joint_positions ) const
{
  std::vector<geometry_msgs::msg::Pose> poses;
  if ( !joint_model_group_->getSolverInstance()->getPositionFK(
           joint_model_group_->getSolverInstance()->getTipFrames(), joint_positions, poses ) ) {
    RCLCPP_ERROR_STREAM( node_->get_logger(), "Computing FK failed." );
    return Eigen::Affine3d::Identity();
  }
  Eigen::Affine3d pose;
  tf2::fromMsg( poses[0], pose );
  return pose;
}

bool InverseKinematics::isCollisionFree(
    const sensor_msgs::msg::JointState &joint_state, const std::vector<double> &solution,
    collision_detection::CollisionResult::ContactMap &contact_map ) const
{
  robot_state_->setVariableValues( joint_state );
  robot_state_->setJointGroupPositions( joint_model_group_, solution );
  planning_scene_->setCurrentState( *robot_state_ );

  collision_detection::CollisionRequest req;
  req.contacts = true;
  collision_detection::CollisionResult res;
  planning_scene_->checkSelfCollision( req, res );

  if ( res.collision ) {
    contact_map = res.contacts;
    for ( auto it = contact_map.begin(); it != contact_map.end(); ++it ) {
      RCLCPP_WARN_STREAM( node_->get_logger(), "Detected collision between '"
                                                   << it->first.first << "' and '"
                                                   << it->first.second << "'." );
    }
  }

  return !res.collision;
}

moveit::core::RobotState
InverseKinematics::getAsRobotState( const sensor_msgs::msg::JointState &joint_state,
                                    const std::vector<double> &solution ) const
{
  robot_state_->setVariableValues( joint_state );
  robot_state_->setJointGroupPositions( joint_model_group_, solution );
  return *robot_state_;
}

std::vector<std::string> InverseKinematics::getGroupJointNames() { return arm_joint_names_; }

std::vector<std::string> InverseKinematics::getAllJointNames() const
{
  return joint_names_;
}

std::string InverseKinematics::getBaseFrame() const
{
  return joint_model_group_->getSolverInstance()->getBaseFrame();
}

std::string InverseKinematics::moveitErrCodeToString( int32_t code )
{
  switch ( code ) {
  case moveit_msgs::msg::MoveItErrorCodes::FAILURE:
    return "FAILURE";
  case moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT:
    return "TIME_OUT";
  case moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION:
    return "NO_IK_SOLUTION";
  default:
    return "";
  }
  return "";
}

std::string InverseKinematics::getParameterFromTopic( const std::string &topic ) const
{
  std::string param;
  auto qos = rclcpp::QoS( 10 );
  qos.transient_local();
  auto subscription = node_->create_subscription<std_msgs::msg::String>(
      topic, qos, [this, topic, &param]( const std_msgs::msg::String::SharedPtr msg ) {
        RCLCPP_INFO( node_->get_logger(), "Received Param msg on topic %s", topic.c_str() );
        param = msg->data;
      } );
  // wait for the message to be received
  rclcpp::Rate rate( 3 );
  int attempt = 0;
  int max_attempt = 20;
  while ( param.empty() && attempt < max_attempt ) {
    rate.sleep();
    attempt++;
  }
  if ( param.empty() )
    RCLCPP_WARN( node_->get_logger(), "Failed to receive param on topic %s", topic.c_str() );
  return param;
}

void InverseKinematics::setKinematicParameters() const
{
  // set this params
  node_->declare_parameter<std::string>( "_kinematics.arm_group.kinematics_solver",
                                         "kdl_kinematics_plugin/KDLKinematicsPlugin" );
  node_->set_parameter( rclcpp::Parameter( "_kinematics.arm_group.kinematics_solver",
                                           "kdl_kinematics_plugin/KDLKinematicsPlugin" ) );
  node_->declare_parameter<double>( "_kinematics.arm_group.kinematics_solver_search_resolution",
                                    0.0050000000000000001 );
  node_->set_parameter(
      rclcpp::Parameter( "_kinematics.arm_group.kinematics_solver_search_resolution", 0.0050000000000000001 ) );
  node_->declare_parameter<double>( "_kinematics.arm_group.kinematics_solver_timeout", 0.050000000000000003 );
  node_->set_parameter(
      rclcpp::Parameter( "_kinematics.arm_group.kinematics_solver_timeout", 0.050000000000000003 ) );

}


} // namespace moveit_twist_controller
