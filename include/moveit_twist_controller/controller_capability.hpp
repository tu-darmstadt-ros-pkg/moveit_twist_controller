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
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  urdf::ModelInterfaceSharedPtr urdf;
  std::string group_name;
  // Commanded arm joints; order/size match candidate_arm_state in isGoalAllowed().
  std::vector<std::string> arm_joint_names;
  // Full active joint chain; order/size match current_joint_angles in isGoalAllowed().
  std::vector<std::string> joint_names;
};

// Pluginlib base class for twist-controller capabilities. A capability inspects each candidate IK
// goal and may veto it (e.g. torque limiting); a vetoed goal is reverted like a collision.
// Plugins are loaded by class name at configure time via the `capabilities` parameter.
class ControllerCapability
{
public:
  virtual ~ControllerCapability() = default;

  // Called once during on_configure(). Return false to abort controller configuration.
  virtual bool initialize( const CapabilityContext &ctx ) = 0;

  // Called every update() cycle. REALTIME: must not allocate, lock, or log without throttling.
  // Return false to veto the candidate, causing the controller to hold the previous goal.
  virtual bool isGoalAllowed( const std::vector<double> &candidate_arm_state,
                              const std::vector<double> &current_joint_angles ) = 0;

  // Human-readable name used in log messages.
  virtual std::string getName() const = 0;
};

} // namespace moveit_twist_controller

#endif // MOVEIT_TWIST_CONTROLLER__CONTROLLER_CAPABILITY_HPP_
