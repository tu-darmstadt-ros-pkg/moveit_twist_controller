#ifndef MOVEIT_TWIST_CONTROLLER__CONTROLLER_CAPABILITY_HPP_
#define MOVEIT_TWIST_CONTROLLER__CONTROLLER_CAPABILITY_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <urdf/model.h>

namespace moveit_twist_controller
{

// Read-only view of the controller state handed to a capability at initialize().
struct CapabilityContext {
  // Controller's lifecycle node, used by capabilities for parameter declaration and logging.
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  // Parsed robot URDF (e.g. to build a dynamics model).
  urdf::ModelInterfaceSharedPtr urdf;
  // Planning group the controller commands.
  std::string group_name;
  // Joints the controller commands. Order and size match the candidate_arm_state passed
  // to isGoalAllowed().
  std::vector<std::string> arm_joint_names;
  // Full active joint chain. Order and size match the current_joint_angles passed to
  // isGoalAllowed().
  std::vector<std::string> joint_names;
};

// Pluginlib base class for twist-controller capabilities. A capability inspects each candidate IK
// goal state the controller produces and may veto it (e.g. torque limiting); a vetoed goal is
// reverted exactly like a collision. Plugins are loaded by class name at configure time via the
// `capabilities` parameter.
class ControllerCapability
{
public:
  virtual ~ControllerCapability() = default;

  // Called once during on_configure(). Return false to abort controller configuration.
  virtual bool initialize( const CapabilityContext &ctx ) = 0;

  // Called every update() cycle. REALTIME context: implementations must not allocate, lock,
  // or log without throttling.
  // candidate_arm_state: proposed goal for the arm joints (same order/size as
  //   ctx.arm_joint_names).
  // current_joint_angles: latest measured state for the full chain (same order/size as
  //   ctx.joint_names).
  // Return false to veto the candidate, causing the controller to hold the previous goal.
  virtual bool isGoalAllowed( const std::vector<double> &candidate_arm_state,
                              const std::vector<double> &current_joint_angles ) = 0;

  // Human-readable name used in log messages.
  virtual std::string getName() const = 0;
};

} // namespace moveit_twist_controller

#endif // MOVEIT_TWIST_CONTROLLER__CONTROLLER_CAPABILITY_HPP_
