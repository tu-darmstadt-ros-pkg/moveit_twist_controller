#ifndef MOVEIT_JOYSTICK_CONTROL__JOYSTICK_CONTROL_HPP_
#define MOVEIT_JOYSTICK_CONTROL__JOYSTICK_CONTROL_HPP_

#include <Eigen/Eigen>
#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "moveit_twist_controller/inverse_kinematics.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <moveit_twist_controller/moveit_twist_controller_parameters.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace moveit_twist_controller
{

struct Twist {
  Eigen::Vector3d linear;
  Eigen::Vector3d angular;
};

struct TwistCommand {
  Twist twist = { Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() };
  rclcpp::Time stamp = rclcpp::Time( 0, 0, RCL_CLOCK_UNINITIALIZED );
};

class MoveitTwistController : public controller_interface::ControllerInterface
{
public:
  MoveitTwistController();
  ~MoveitTwistController() override = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn
  on_configure( const rclcpp_lifecycle::State &previous_state ) override;
  controller_interface::CallbackReturn
  on_activate( const rclcpp_lifecycle::State &previous_state ) override;
  controller_interface::CallbackReturn
  on_deactivate( const rclcpp_lifecycle::State &previous_state ) override;

  controller_interface::return_type update( const rclcpp::Time &time,
                                            const rclcpp::Duration &period ) override;

private:
  void publishStatus() const;
  void updateArm( const rclcpp::Time &time, const rclcpp::Duration &period );
  bool computeNewGoalPose( const rclcpp::Duration &period );

  bool resetPoseCb( const std_srvs::srv::Empty::Request::SharedPtr request,
                    std_srvs::srv::Empty::Response::SharedPtr response );
  bool resetToolCenterCb( const std_srvs::srv::Empty::Request::SharedPtr request,
                          std_srvs::srv::Empty::Response::SharedPtr response );
  bool holdPoseCb( const std_srvs::srv::SetBool::Request::SharedPtr request,
                   std_srvs::srv::SetBool::Response::SharedPtr response );
  bool moveToolCenterCb( const std_srvs::srv::SetBool::Request::SharedPtr request,
                         std_srvs::srv::SetBool::Response::SharedPtr response );
  void publishCollisionMarkers();
  void setupInterfaces();
  static double computeJointAngleDiff( double angle_1, double angle_2 );
  bool setCommand( double value, const std::string &joint_name, const std::string &interface_name );
  bool getState( double &value, const std::string &joint_name,
                 const std::string &interface_name ) const;
  /// Transforms pose to desired frame
  /// Pose has to be relative to base frame
  geometry_msgs::msg::PoseStamped getPoseInFrame( const Eigen::Affine3d &pose,
                                                  const std::string &frame );
  bool calculateInverseKinematicsConsideringVelocityLimits( Eigen::Affine3d &new_eef_pose,
                                                            const Eigen::Affine3d &old_eef_pose,
                                                            const rclcpp::Duration &period );

  bool initialized_ = false;
  bool enabled_ = false;
  std::atomic<bool> reset_pose_ = false;
  std::atomic<bool> reset_tool_center_ = false;
  std::atomic<bool> move_tool_center_ = false;

  int free_angle_ = -1;

  Eigen::Affine3d tool_center_offset_;
  Eigen::Affine3d tool_goal_pose_;
  Eigen::Affine3d ee_goal_pose_;
  std::vector<double> goal_state_;
  std::vector<double> previous_goal_state_;

  collision_detection::CollisionResult::ContactMap contact_map_;

  realtime_tools::RealtimeBuffer<TwistCommand> twist_cmd_buf_;
  Twist twist_;
  std::vector<double> joint_velocity_limits_;
  std::vector<double> smoothed_joint_velocities_;

  std::vector<std::string> joint_names_;
  std::vector<std::string> arm_joint_names_;
  bool joint_state_received_ = false;
  std::vector<double> current_joint_angles_;
  std::vector<double> current_arm_joint_angles;
  std::vector<int> arm_joint_indices_; // indices of arm joints in joint_names_

  InverseKinematics ik_;
  std::atomic<bool> hold_pose_ = false;
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::PoseStamped> hold_goal_pose_buf_;

  Params params_;
  std::shared_ptr<ParamListener> param_listener_;
  mutable std::map<std::string, size_t> joint_state_interface_mapping_;
  mutable std::map<std::string, size_t> joint_command_interface_mapping_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_sub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_pose_server_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_tool_center_server_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr hold_pose_server_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr move_tool_center_server_;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr collision_marker_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<visualization_msgs::msg::MarkerArray>>
      rt_collision_marker_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr enabled_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Node::SharedPtr moveit_init_node_;
};

} // namespace moveit_twist_controller

#endif // MOVEIT_JOYSTICK_CONTROL__JOYSTICK_CONTROL_HPP_
