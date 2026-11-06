#include <gmock/gmock.h>

#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>

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

int main( int argc, char **argv )
{
  ::testing::InitGoogleMock( &argc, argv );
  rclcpp::init( argc, argv );
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
