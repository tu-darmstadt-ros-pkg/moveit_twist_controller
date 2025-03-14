#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

namespace moveit_twist_controller
{

class InverseKinematics
{
public:
  InverseKinematics() : joint_model_group_( nullptr ) { }

  bool init( rclcpp_lifecycle::LifecycleNode::SharedPtr lifecycle_node,
             rclcpp::Node::SharedPtr node, const std::string &group_name );
  bool calcInvKin( const Eigen::Affine3d &pose, const std::vector<double> &seed,
                   std::vector<double> &solution );
  Eigen::Affine3d getEndEffectorPose( const std::vector<double> &joint_positions ) const;

  bool isCollisionFree( const sensor_msgs::msg::JointState &joint_state,
                        const std::vector<double> &solution,
                        collision_detection::CollisionResult::ContactMap &contact_map ) const;
  moveit::core::RobotState getAsRobotState( const sensor_msgs::msg::JointState &joint_state,
                                            const std::vector<double> &solution ) const;

  std::vector<std::string> getGroupJointNames();
  std::vector<std::string> getAllJointNames() const;
  std::string getBaseFrame() const;
  bool getJointLimits( const std::string &joint_name, double &lower, double &upper ) const;
  std::vector<double> getJointVelocityLimits() const;

private:
  std::string moveitErrCodeToString( int32_t code );
  std::string getParameterFromTopic( const std::string &topic, double timeout_s ) const;
  void setKinematicParameters( const std::string &group_name ) const;
  template<typename T>
  void declareAndSetMoveitParameter( const std::string &param_name, const std::string &group_name,
                                     const T &default_value ) const;

  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;
  planning_scene::PlanningScenePtr planning_scene_;
  const moveit::core::JointModelGroup *joint_model_group_;

  std::string tip_frame_;
  std::vector<std::string> arm_joint_names_;
  std::vector<std::string> joint_names_;
  rclcpp::Node::SharedPtr node_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr lifecycle_node_;
};

template<typename T>
void InverseKinematics::declareAndSetMoveitParameter( const std::string &param_name,
                                                      const std::string &group_name,
                                                      const T &default_value ) const
{
  // declare the parameter with a default value on lifecycle node
  lifecycle_node_->declare_parameter<T>( param_name, default_value );
  T value = lifecycle_node_->get_parameter( param_name ).get_value<T>();

  std::string moveit_param_name = "_kinematics." + group_name + "." + param_name;
  // declare and set the parameter on the node
  node_->declare_parameter<T>( moveit_param_name, value );
  node_->set_parameter( rclcpp::Parameter( moveit_param_name, value ) );
}

} // namespace moveit_twist_controller
#endif
