#ifndef MOVEIT_TWIST_CONTROLLER__CONTROLLER_CAPABILITY_HPP_
#define MOVEIT_TWIST_CONTROLLER__CONTROLLER_CAPABILITY_HPP_

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <urdf/model.h>

#include <string>
#include <vector>

namespace moveit_twist_controller
{

// Read-only context handed to a capability at configure time. It exposes the pieces of the
// controller a capability typically needs (node for parameters/logging, robot URDF, group and
// joint naming) without granting access to the controller internals.
struct CapabilityContext {
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  urdf::ModelInterfaceSharedPtr urdf;
  std::string group_name;
  std::vector<std::string> arm_joint_names;
  std::vector<std::string> joint_names;
};

// Pluginlib base class for twist-controller capabilities. A capability may veto candidate IK goal
// states (e.g. torque limiting); a vetoed goal is reverted exactly like a collision. Plugins are
// loaded by class name at configure time via the `capabilities` parameter.
class ControllerCapability
{
public:
  virtual ~ControllerCapability() = default;

  // Called once at configure time. Return false to abort controller configuration.
  virtual bool initialize( const CapabilityContext &ctx ) = 0;

  // Called in the update loop for each candidate goal. Return false to veto the goal, in which
  // case the controller reverts to the previous goal state. Must be realtime-safe.
  virtual bool isGoalAllowed( const std::vector<double> &candidate_arm_state,
                              const std::vector<double> &current_joint_angles ) = 0;

  // Human-readable name used in log messages.
  virtual std::string getName() const = 0;
};

} // namespace moveit_twist_controller

#endif // MOVEIT_TWIST_CONTROLLER__CONTROLLER_CAPABILITY_HPP_
