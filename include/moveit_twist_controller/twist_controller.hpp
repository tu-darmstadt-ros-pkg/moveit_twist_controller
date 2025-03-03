#ifndef MOVEIT_JOYSTICK_CONTROL__JOYSTICK_CONTROL_HPP_
#define MOVEIT_JOYSTICK_CONTROL__JOYSTICK_CONTROL_HPP_

#include <memory>
#include <vector>
#include <string>
#include <Eigen/Eigen>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "moveit_twist_controller/inverse_kinematics.hpp"
#include <std_msgs/msg/bool.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_twist_controller/moveit_twist_controller_parameters.hpp>

namespace moveit_twist_controller {

struct Twist {
  Eigen::Vector3d linear;
  Eigen::Vector3d angular;
};

class MoveitTwistController : public controller_interface::ControllerInterface {
public:
  MoveitTwistController();
  ~MoveitTwistController() override = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void publishStatus() const;
  bool loadGripperJointLimits();
  void updateArm(const rclcpp::Time & time, const rclcpp::Duration & period);
  bool computeNewGoalPose(const rclcpp::Duration & period);
  void updateGripper(const rclcpp::Time & time, const rclcpp::Duration & period);

  void twistCmdCb(const geometry_msgs::msg::Twist::SharedPtr twist_msg);
  void gripperCmdCb(const std_msgs::msg::Float64::SharedPtr float_ptr);
  bool resetPoseCb(const std_srvs::srv::Empty::Request::SharedPtr request,
                   std_srvs::srv::Empty::Response::SharedPtr response);
  bool resetToolCenterCb(const std_srvs::srv::Empty::Request::SharedPtr request,
                         std_srvs::srv::Empty::Response::SharedPtr response);
  bool holdPoseCb(const std_srvs::srv::SetBool::Request::SharedPtr request,
                  std_srvs::srv::SetBool::Response::SharedPtr response);
  bool moveToolCenterCb(const std_srvs::srv::SetBool::Request::SharedPtr request,
                        std_srvs::srv::SetBool::Response::SharedPtr response);
  void publishRobotState(const std::vector<double>& arm_joint_states, const collision_detection::CollisionResult::ContactMap& contact_map_);
  void hideRobotState() const;
  /// Transforms pose to desired frame
  /// Pose has to be relative to base frame
  geometry_msgs::msg::PoseStamped getPoseInFrame(const Eigen::Affine3d& pose,
                                                  const std::string &frame );


  bool initialized_ = false;
  bool enabled_ ;
  bool reset_pose_;
  bool reset_tool_center_;
  bool move_tool_center_;

  double max_speed_gripper_;
  int free_angle_;

  Eigen::Affine3d tool_center_offset_;
  Eigen::Affine3d tool_goal_pose_;
  Eigen::Affine3d ee_goal_pose_;
  std::vector<double> goal_state_;
  std::vector<double> previous_goal_state_;

  collision_detection::CollisionResult::ContactMap contact_map_;

  Twist twist_;
  double gripper_pos_;
  double gripper_speed_;

  std::vector<std::string> joint_names_;
  std::vector<std::string> arm_joint_names_;
  bool joint_state_received_;
  std::vector<double> current_joint_angles_;
  std::vector<double> current_arm_joint_angles;
  std::vector<int> arm_joint_indices_;

  std::string gripper_joint_name_;
  double gripper_upper_limit_;
  double gripper_lower_limit_;

  InverseKinematics ik_;
  bool hold_pose_;
  geometry_msgs::msg::PoseStamped hold_goal_pose_;

  std::thread status_thread_;
  rclcpp::Executor::SharedPtr executor_;
  rclcpp::Node::SharedPtr status_node_;



  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gripper_cmd_sub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_pose_server_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_tool_center_server_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr hold_pose_server_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr move_tool_center_server_;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr enabled_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<moveit_twist_controller::ParamListener> param_listener_;
};

}  // namespace moveit_twist_controller

#endif  // MOVEIT_JOYSTICK_CONTROL__JOYSTICK_CONTROL_HPP_
