//
// Created by aljoscha-schmidt on 2/20/25.
//
#include <gmock/gmock.h>
#include <memory>

#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/utilities.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

TEST(TestLoadMoveitTwistController, load_controller)
{
  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  controller_manager::ControllerManager cm(
    executor, ros2_control_test_assets::minimal_robot_urdf, true, "test_controller_manager");
  const std::string test_file_path =
    std::string(TEST_FILES_DIRECTORY) + "/test_moveit_twist_controller_params.yaml";

  cm.set_parameter({"test_moveit_twist_controller.params_file", test_file_path});
  cm.set_parameter(
    {"test_moveit_twist_controller.type",
     "moveit_twist_controller/MoveitTwistController"});

  ASSERT_NE(cm.load_controller("test_moveit_twist_controller"), nullptr);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}