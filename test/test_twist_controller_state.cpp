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
#include <hardware_interface/loaned_state_interface.hpp>

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
