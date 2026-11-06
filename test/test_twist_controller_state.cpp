#include <gmock/gmock.h>

#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <vector>

#define private public
#define protected public
#include <moveit_twist_controller/twist_controller.hpp>
#undef protected
#undef private

#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp/rclcpp.hpp>

// A state handle locked by another thread is contention, not a reason to throw from update()
TEST( MoveitTwistControllerState, ABusyStateReadsAsUnavailable )
{
  moveit_twist_controller::MoveitTwistController controller;
  auto handle =
      std::make_shared<hardware_interface::StateInterface>( "joint1", "position", "double", "0.3" );
  controller.state_interfaces_.emplace_back( handle );
  controller.joint_state_interface_mapping_["joint1/position"] = 0;

  double value = 0.0;
  ASSERT_TRUE( controller.getState( value, "joint1", "position" ) );
  EXPECT_DOUBLE_EQ( value, 0.3 );

  std::unique_lock<std::shared_mutex> busy( handle->get_mutex() );
  value = 0.0;
  EXPECT_FALSE( controller.getState( value, "joint1", "position" ) );
  EXPECT_DOUBLE_EQ( value, 0.0 ) << "a failed read overwrote the value";
}

// Under async hardware a busy state is transient, so the controller skips the cycle and the
// commands of the last cycle hold. An error would stop the whole arm chain.
TEST( MoveitTwistControllerState, ABusyStateSkipsTheCycleAndKeepsTheCommands )
{
  moveit_twist_controller::MoveitTwistController controller;
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = "test_moveit_twist_controller";
  params.update_rate = 50;
  params.node_options.arguments( { "--ros-args", "--params-file",
                                   TEST_FILES_DIRECTORY
                                   "/test_moveit_twist_controller_params.yaml" } );
  ASSERT_EQ( controller.init( params ), controller_interface::return_type::OK );

  controller.initialized_ = true;
  controller.joint_names_ = { "joint1" };
  controller.current_joint_angles_.resize( 1 );
  auto state =
      std::make_shared<hardware_interface::StateInterface>( "joint1", "position", "double", "0.3" );
  controller.state_interfaces_.emplace_back( state );
  controller.joint_state_interface_mapping_["joint1/position"] = 0;
  auto command = std::make_shared<hardware_interface::CommandInterface>( "joint1", "position",
                                                                         "double", "0.7" );
  controller.command_interfaces_.emplace_back( command );
  controller.joint_command_interface_mapping_["joint1/position"] = 0;

  {
    std::unique_lock<std::shared_mutex> busy( state->get_mutex() );
    EXPECT_EQ( controller.update( rclcpp::Time( 0, 0 ), rclcpp::Duration::from_seconds( 0.02 ) ),
               controller_interface::return_type::OK );
  }
  EXPECT_DOUBLE_EQ( command->get_optional().value(), 0.7 ) << "the skipped cycle wrote a command";
}

// Commands are stamped with the node clock, while hcm and ros2_control 6.12 run update() on a
// steady trigger clock outside simulation
TEST( MoveitTwistControllerState, AFreshCommandIsTakenOnTheNodeClock )
{
  moveit_twist_controller::MoveitTwistController controller;
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = "test_moveit_twist_controller";
  params.update_rate = 50;
  params.node_options.arguments( { "--ros-args", "--params-file",
                                   TEST_FILES_DIRECTORY
                                   "/test_moveit_twist_controller_params.yaml" } );
  ASSERT_EQ( controller.init( params ), controller_interface::return_type::OK );
  controller.params_ = controller.param_listener_->get_params();
  controller.joint_cmd_velocity_.assign( 1, 0.0 );
  // As on_activate leaves the buffers: zero commands stamped long ago
  const auto clock_type = controller.get_node()->get_clock()->get_clock_type();
  moveit_twist_controller::TwistCommand twist;
  twist.stamp = rclcpp::Time( 0, 0, clock_type );
  controller.twist_cmd_buf_.writeFromNonRT( twist );
  moveit_twist_controller::NullspaceCommand nullspace;
  nullspace.stamp = rclcpp::Time( 0, 0, clock_type );
  controller.nullspace_cmd_buf_.writeFromNonRT( nullspace );

  moveit_twist_controller::JointCommand command;
  command.velocity = { 0.5 };
  command.stamp = controller.get_node()->now();
  controller.joint_cmd_buf_.writeFromNonRT( command );
  controller.readCommands();
  EXPECT_EQ( controller.joint_cmd_velocity_, std::vector<double>{ 0.5 } );
}

// Under hcm the update time comes from a steady clock, which TF-based displays cannot place
TEST( MoveitTwistControllerState, TheGoalPoseIsStampedWithTheNodeClock )
{
  moveit_twist_controller::MoveitTwistController controller;
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = "test_moveit_twist_controller";
  params.update_rate = 50;
  params.node_options.arguments( { "--ros-args", "--params-file",
                                   TEST_FILES_DIRECTORY
                                   "/test_moveit_twist_controller_params.yaml" } );
  ASSERT_EQ( controller.init( params ), controller_interface::return_type::OK );

  const auto goal_pose = controller.goalPose( "arm_base_link" );
  EXPECT_EQ( goal_pose.header.frame_id, "arm_base_link" );
  const rclcpp::Time now = controller.get_node()->now();
  EXPECT_NEAR( ( now - rclcpp::Time( goal_pose.header.stamp, now.get_clock_type() ) ).seconds(),
               0.0, 1.0 );
}

int main( int argc, char **argv )
{
  ::testing::InitGoogleMock( &argc, argv );
  rclcpp::init( argc, argv );
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
