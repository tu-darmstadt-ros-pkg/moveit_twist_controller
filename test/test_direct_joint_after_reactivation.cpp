#include <gmock/gmock.h>

#include <controller_interface/controller_interface_params.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "moveit_twist_controller/twist_controller.hpp"

#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace
{
const char kUrdf[] = R"(
<robot name="two_link">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="link2"/>
  <link name="tip"/>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.0" upper="3.0" effort="10" velocity="1.0"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0 0 0.3"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.0" upper="3.0" effort="10" velocity="1.0"/>
  </joint>
  <joint name="tip_joint" type="fixed">
    <parent link="link2"/><child link="tip"/>
    <origin xyz="0 0 0.3"/>
  </joint>
</robot>
)";

const char kSrdf[] = R"(
<robot name="two_link">
  <group name="arm_group"><chain base_link="base_link" tip_link="tip"/></group>
</robot>
)";
} // namespace

class DirectJointAfterReactivationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    controller_ = std::make_shared<moveit_twist_controller::MoveitTwistController>();
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "twist_controller";
    params.robot_description = kUrdf;
    params.update_rate = 100;
    params.controller_manager_update_rate = 100;
    params.node_options = rclcpp::NodeOptions()
                              .allow_undeclared_parameters( true )
                              .automatically_declare_parameters_from_overrides( true )
                              .parameter_overrides( { { "robot_description_semantic", kSrdf },
                                                      { "group_name", "arm_group" } } );
    ASSERT_EQ( controller_->init( params ), controller_interface::return_type::OK );
    ASSERT_EQ( controller_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE );

    std::vector<hardware_interface::LoanedCommandInterface> commands;
    for ( const auto &name : controller_->command_interface_configuration().names ) {
      auto itf = std::make_shared<hardware_interface::CommandInterface>( describe( name ) );
      command_interfaces_[name.substr( 0, name.find( '/' ) )] = itf;
      commands.emplace_back( itf, nullptr );
    }
    std::vector<hardware_interface::LoanedStateInterface> states;
    for ( const auto &name : controller_->state_interface_configuration().names ) {
      auto itf = std::make_shared<hardware_interface::StateInterface>( describe( name ) );
      state_interfaces_[name.substr( 0, name.find( '/' ) )] = itf;
      states.emplace_back( itf, nullptr );
    }
    controller_->assign_interfaces( std::move( commands ), std::move( states ) );

    executor_.add_node( controller_->get_node()->get_node_base_interface() );
    publisher_node_ = std::make_shared<rclcpp::Node>( "joint_cmd_publisher" );
    executor_.add_node( publisher_node_ );
    joint_cmd_pub_ = publisher_node_->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/twist_controller/joint_cmd", 10 );
  }

  void TearDown() override
  {
    executor_.cancel();
    controller_.reset();
  }

  static hardware_interface::InterfaceDescription describe( const std::string &name )
  {
    const auto slash = name.find( '/' );
    hardware_interface::InterfaceInfo info;
    info.name = name.substr( slash + 1 );
    info.initial_value = "0.0";
    return hardware_interface::InterfaceDescription( name.substr( 0, slash ), info );
  }

  void setState( double joint1, double joint2 )
  {
    ASSERT_TRUE( state_interfaces_["joint1"]->set_value( joint1 ) );
    ASSERT_TRUE( state_interfaces_["joint2"]->set_value( joint2 ) );
  }

  double command( const std::string &joint )
  {
    return command_interfaces_[joint]->get_optional<double>().value_or( std::nan( "" ) );
  }

  void update()
  {
    ASSERT_EQ( controller_->update( controller_->get_node()->now(), rclcpp::Duration::from_seconds( 0.01 ) ),
               controller_interface::return_type::OK );
  }

  // Sends a single-joint jog command and waits until the controller received it.
  void sendJointCommand( const std::vector<double> &velocities )
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = velocities;
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while ( joint_cmd_pub_->get_subscription_count() == 0 && std::chrono::steady_clock::now() < deadline ) {
      executor_.spin_some();
      std::this_thread::sleep_for( 10ms );
    }
    for ( int i = 0; i < 5; ++i ) {
      joint_cmd_pub_->publish( msg );
      executor_.spin_some();
      std::this_thread::sleep_for( 10ms );
    }
    executor_.spin_some();
  }

  std::shared_ptr<moveit_twist_controller::MoveitTwistController> controller_;
  std::map<std::string, hardware_interface::CommandInterface::SharedPtr> command_interfaces_;
  std::map<std::string, hardware_interface::StateInterface::SharedPtr> state_interfaces_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Node::SharedPtr publisher_node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_pub_;
};

// Teleop leaves the arm at A, another controller (e.g. a MoveIt pose) moves it to B while this
// controller is inactive. The first single-joint jog after re-activation must start at B instead
// of the stale goal A.
TEST_F( DirectJointAfterReactivationTest, JoggingStartsFromCurrentStateAfterReactivation )
{
  setState( 0.5, 0.5 );
  ASSERT_EQ( controller_->get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE );
  for ( int i = 0; i < 3; ++i ) update();
  EXPECT_NEAR( command( "joint1" ), 0.5, 1e-3 );
  EXPECT_NEAR( command( "joint2" ), 0.5, 1e-3 );

  ASSERT_EQ( controller_->get_node()->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE );
  setState( -0.5, -0.5 );
  ASSERT_EQ( controller_->get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE );

  sendJointCommand( { 0.1, 0.0 } );
  update();

  // one 10 ms step at 0.1 rad/s from B, not from A
  EXPECT_NEAR( command( "joint1" ), -0.5 + 0.001, 1e-3 );
  EXPECT_NEAR( command( "joint2" ), -0.5, 1e-3 );
}

int main( int argc, char **argv )
{
  ::testing::InitGoogleTest( &argc, argv );
  rclcpp::init( argc, argv );
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
