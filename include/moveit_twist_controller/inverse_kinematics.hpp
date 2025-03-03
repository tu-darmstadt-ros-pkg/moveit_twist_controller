#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include<rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/planning_scene/planning_scene.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

namespace moveit_twist_controller {

class InverseKinematics {
public:
  InverseKinematics(  )
      : joint_model_group_( nullptr )
  {
  }

  bool init(rclcpp_lifecycle::LifecycleNode::SharedPtr lifecycle_node, rclcpp::Node::SharedPtr node, const std::string& group_name);
  bool calcInvKin(const Eigen::Affine3d &pose, const std::vector<double>& seed, std::vector<double> &solution);
  Eigen::Affine3d getEndEffectorPose(const std::vector<double>& joint_positions ) const;

  bool isCollisionFree(const sensor_msgs::msg::JointState& joint_state, const std::vector<double>& solution, collision_detection::CollisionResult::ContactMap& contact_map ) const;
  moveit::core::RobotState getAsRobotState(const sensor_msgs::msg::JointState& joint_state, const std::vector<double>& solution ) const;

  std::vector<std::string> getGroupJointNames();
  std::vector<std::string> getAllJointNames() const;
  std::string getBaseFrame() const;
  bool getJointLimits(const std::string& joint_name, double& lower, double& upper) const;
private:
  std::string moveitErrCodeToString(int32_t code);
  std::string getParameterFromTopic(const std::string& topic ) const;
  void setKinematicParameters() const;

  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;
  planning_scene::PlanningScenePtr planning_scene_;
  const moveit::core::JointModelGroup* joint_model_group_;

  std::string tip_frame_;
  std::vector<std::string> arm_joint_names_;
  std::vector<std::string> joint_names_;
  rclcpp::Node::SharedPtr node_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr lifecycle_node_;
};

}
#endif
