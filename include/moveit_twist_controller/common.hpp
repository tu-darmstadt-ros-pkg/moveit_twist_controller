#ifndef MOVEIT_JOYSTICK_CONTROL_COMMON_H
#define MOVEIT_JOYSTICK_CONTROL_COMMON_H

#include <Eigen/Eigen>
#include <sensor_msgs/msg/joint_state.hpp>

#include <moveit/robot_state/robot_state.hpp>

namespace moveit_twist_controller
{

Eigen::Quaterniond rpyToRot( const Eigen::Vector3d &rpy );
Eigen::Vector3d rotToRpy( const Eigen::Matrix3d &rot );
// std::vector<double> stateFromList(const sensor_msgs::msg::JointState& last_state, const std::vector<std::string>& joint_names);

void updateRobotStatePose( moveit::core::RobotState &state, const Eigen::Affine3d &pose );

} // namespace moveit_twist_controller

#endif // MOVEIT_JOYSTICK_CONTROL_COMMON_H
