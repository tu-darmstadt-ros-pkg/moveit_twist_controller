#include <gmock/gmock.h>

#include <memory>
#include <string>

#include <moveit_twist_controller/inverse_kinematics.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

// The controller manager passes the SRDF as a parameter override of the controller's node
TEST( SemanticDescription, ComesFromTheParameterOverride )
{
  const std::string srdf = "<robot name=\"test_robot\"/>";
  rclcpp::NodeOptions options;
  options.parameter_overrides( { rclcpp::Parameter( "robot_description_semantic", srdf ) } );
  const auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>( "twist", options );
  EXPECT_EQ( moveit_twist_controller::semanticDescriptionParameter( node ), srdf );
}

int main( int argc, char **argv )
{
  ::testing::InitGoogleTest( &argc, argv );
  rclcpp::init( argc, argv );
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
