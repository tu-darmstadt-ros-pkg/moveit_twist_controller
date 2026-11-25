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
#include <moveit/kinematics_base/kinematics_base.hpp>
#include <moveit/utils/robot_model_test_utils.hpp>
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
  // Zero commands stamped long ago, as on_activate leaves the buffers
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

// A solver for a single prismatic joint along x that counts its solves
class CountingSolver : public kinematics::KinematicsBase
{
public:
  explicit CountingSolver( const std::vector<std::string> &joint_names )
      : joint_names_( joint_names )
  {
    base_frame_ = "base";
    tip_frames_ = { "tip" };
  }

  bool getPositionIK( const geometry_msgs::msg::Pose &, const std::vector<double> &,
                      std::vector<double> &, moveit_msgs::msg::MoveItErrorCodes &,
                      const kinematics::KinematicsQueryOptions & ) const override
  {
    return false;
  }
  bool searchPositionIK( const geometry_msgs::msg::Pose &pose, const std::vector<double> &, double,
                         std::vector<double> &solution, moveit_msgs::msg::MoveItErrorCodes &,
                         const kinematics::KinematicsQueryOptions & ) const override
  {
    ++solves;
    solution = { pose.position.x };
    return true;
  }
  bool searchPositionIK( const geometry_msgs::msg::Pose &, const std::vector<double> &, double,
                         const std::vector<double> &, std::vector<double> &,
                         moveit_msgs::msg::MoveItErrorCodes &,
                         const kinematics::KinematicsQueryOptions & ) const override
  {
    return false;
  }
  bool searchPositionIK( const geometry_msgs::msg::Pose &, const std::vector<double> &, double,
                         std::vector<double> &, const IKCallbackFn &,
                         moveit_msgs::msg::MoveItErrorCodes &,
                         const kinematics::KinematicsQueryOptions & ) const override
  {
    return false;
  }
  bool searchPositionIK( const geometry_msgs::msg::Pose &, const std::vector<double> &, double,
                         const std::vector<double> &, std::vector<double> &, const IKCallbackFn &,
                         moveit_msgs::msg::MoveItErrorCodes &,
                         const kinematics::KinematicsQueryOptions & ) const override
  {
    return false;
  }
  bool getPositionFK( const std::vector<std::string> &, const std::vector<double> &joint_angles,
                      std::vector<geometry_msgs::msg::Pose> &poses ) const override
  {
    poses.assign( 1, geometry_msgs::msg::Pose() );
    poses[0].position.x = joint_angles[0];
    return true;
  }
  const std::vector<std::string> &getJointNames() const override { return joint_names_; }
  const std::vector<std::string> &getLinkNames() const override { return tip_frames_; }

  mutable int solves = 0;

private:
  std::vector<std::string> joint_names_;
};

// An active controller on a one joint arm whose IK is a CountingSolver
class ActiveController : public ::testing::Test
{
protected:
  void SetUp() override
  {
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "test_moveit_twist_controller";
    params.update_rate = 50;
    params.node_options.arguments( { "--ros-args", "--params-file",
                                     TEST_FILES_DIRECTORY
                                     "/test_moveit_twist_controller_params.yaml" } );
    ASSERT_EQ( controller.init( params ), controller_interface::return_type::OK );
    controller.params_ = controller.param_listener_->get_params();

    moveit::core::RobotModelBuilder builder( "arm", "base" );
    builder.addChain( "base->tip", "prismatic" );
    builder.addGroupChain( "base", "tip", "arm" );
    const auto model = builder.build();
    ASSERT_TRUE( model );
    auto *group = model->getJointModelGroup( "arm" );
    const auto joint_names = group->getActiveJointModelNames();
    group->setSolverAllocators( [this, joint_names]( const moveit::core::JointModelGroup * ) {
      auto solver = std::make_shared<CountingSolver>( joint_names );
      solver_ = solver.get();
      return solver;
    } );
    ASSERT_NE( solver_, nullptr );

    auto &ik = controller.ik_;
    ik.robot_model_ = model;
    ik.joint_model_group_ = group;
    ik.robot_state_ = std::make_shared<moveit::core::RobotState>( model );
    ik.planning_scene_ = std::make_shared<planning_scene::PlanningScene>( model );
    ik.arm_joint_names_ = joint_names;
    ik.node_ = rclcpp::Node::make_shared( "counting_ik" );

    controller.joint_names_ = joint_names;
    controller.arm_joint_names_ = joint_names;
    controller.arm_joint_indices_ = { 0 };
    for ( auto *v : { &controller.current_joint_angles_, &controller.current_arm_joint_angles,
                      &controller.goal_state_, &controller.previous_goal_state_, &controller.ik_seed_,
                      &controller.nullspace_bias_, &controller.smoothed_joint_velocities_ } )
      v->assign( 1, 0.0 );
    controller.joint_velocity_limits_ = { 1.0 };
    controller.arm_joint_lower_limits_ = { -1.0 };
    controller.arm_joint_upper_limits_ = { 1.0 };

    state_ = std::make_shared<hardware_interface::StateInterface>( joint_names[0], "position",
                                                                   "double", "0.3" );
    controller.state_interfaces_.emplace_back( state_ );
    controller.joint_state_interface_mapping_[joint_names[0] + "/position"] = 0;
    command_ = std::make_shared<hardware_interface::CommandInterface>( joint_names[0], "position",
                                                                       "double", "0.0" );
    controller.command_interfaces_.emplace_back( command_ );
    controller.joint_command_interface_mapping_[joint_names[0] + "/position"] = 0;

    const auto node = controller.get_node();
    controller.goal_pose_pub_ =
        node->create_publisher<geometry_msgs::msg::PoseStamped>( "~/goal_pose", 10 );
    controller.collision_marker_pub_ =
        node->create_publisher<visualization_msgs::msg::MarkerArray>( "~/collision_markers", 10 );
    controller.rt_collision_marker_pub_ =
        std::make_unique<realtime_tools::RealtimePublisher<visualization_msgs::msg::MarkerArray>>(
            controller.collision_marker_pub_ );
    controller.enabled_pub_ = node->create_publisher<std_msgs::msg::Bool>( "~/enabled", 10 );
    ASSERT_EQ( controller.on_activate( rclcpp_lifecycle::State() ),
               controller_interface::CallbackReturn::SUCCESS );
  }

  void update()
  {
    ASSERT_EQ( controller.update( rclcpp::Time( 0, 0 ), rclcpp::Duration::from_seconds( 0.02 ) ),
               controller_interface::return_type::OK );
  }

  void sendTwist( const double linear_x, const rclcpp::Time &stamp )
  {
    moveit_twist_controller::TwistCommand twist;
    twist.twist.linear = Eigen::Vector3d( linear_x, 0.0, 0.0 );
    twist.twist.angular = Eigen::Vector3d::Zero();
    twist.stamp = stamp;
    controller.twist_cmd_buf_.writeFromNonRT( twist );
  }

  double commanded() const { return command_->get_optional().value(); }

  moveit_twist_controller::MoveitTwistController controller;
  CountingSolver *solver_ = nullptr;
  std::shared_ptr<hardware_interface::StateInterface> state_;
  std::shared_ptr<hardware_interface::CommandInterface> command_;
};

// Each solve may take 10 ms on the RT thread, so without input update() keeps the last goal
TEST_F( ActiveController, DoesNotSolveWithoutInput )
{
  update();
  EXPECT_EQ( solver_->solves, 1 ) << "the first cycle did not ground the goal on the joint state";
  EXPECT_DOUBLE_EQ( commanded(), 0.3 );

  const rclcpp::Time now = controller.get_node()->now();
  sendTwist( 0.1, now - rclcpp::Duration::from_seconds( 10.0 * controller.params_.cmd_timeout ) );
  for ( int i = 0; i < 10; ++i ) update();
  EXPECT_EQ( solver_->solves, 1 );
  EXPECT_DOUBLE_EQ( commanded(), 0.3 );
}

TEST_F( ActiveController, SolvesInTheCycleACommandArrivesAfterIdle )
{
  for ( int i = 0; i < 5; ++i ) update();
  const int idle_solves = solver_->solves;

  sendTwist( 0.1, controller.get_node()->now() );
  update();
  EXPECT_GT( solver_->solves, idle_solves );
  EXPECT_NEAR( commanded(), 0.3 + 0.1 * 0.02, 1e-9 );
}

// A held goal lives in another frame, so moving that frame must move the arm without any input
TEST_F( ActiveController, HoldModeSolvesWhenTheHeldFrameMoves )
{
  update();
  geometry_msgs::msg::TransformStamped base_to_odom;
  base_to_odom.header.frame_id = "base";
  base_to_odom.child_frame_id = "odom";
  base_to_odom.transform.rotation.w = 1.0;
  ASSERT_TRUE( controller.tf_buffer_->setTransform( base_to_odom, "test", true ) );
  geometry_msgs::msg::PoseStamped hold_goal;
  hold_goal.header.frame_id = "odom";
  hold_goal.pose.position.x = 0.3;
  controller.hold_goal_pose_buf_.writeFromNonRT( hold_goal );
  controller.hold_pose_.store( true );
  for ( int i = 0; i < 5; ++i ) update();
  const int held_solves = solver_->solves;
  EXPECT_DOUBLE_EQ( commanded(), 0.3 );

  base_to_odom.transform.translation.x = 0.01;
  ASSERT_TRUE( controller.tf_buffer_->setTransform( base_to_odom, "test", true ) );
  update();
  EXPECT_GT( solver_->solves, held_solves );
  EXPECT_NEAR( commanded(), 0.31, 1e-9 );
}

// With smoothing the arm coasts after the input ends, so the solve must go on until it stops
TEST_F( ActiveController, SmoothingKeepsSolvingUntilTheCoastEnds )
{
  controller.get_node()->set_parameter( rclcpp::Parameter( "joint_velocity_smoothing_factor", 0.5 ) );
  update();
  for ( int i = 0; i < 5; ++i ) {
    sendTwist( 0.5, controller.get_node()->now() );
    update();
  }

  sendTwist( 0.0, controller.get_node()->now() );
  const int solves_at_release = solver_->solves;
  const double position_at_release = commanded();
  update();
  EXPECT_GT( solver_->solves, solves_at_release );
  EXPECT_GT( commanded(), position_at_release ) << "the arm did not coast";

  int cycles = 0;
  while ( controller.smoothed_joint_velocities_[0] != 0.0 && cycles++ < 2000 ) update();
  ASSERT_EQ( controller.smoothed_joint_velocities_[0], 0.0 ) << "the coast did not end";
  const int solves_at_rest = solver_->solves;
  for ( int i = 0; i < 5; ++i ) update();
  EXPECT_EQ( solver_->solves, solves_at_rest );
}

int main( int argc, char **argv )
{
  ::testing::InitGoogleMock( &argc, argv );
  rclcpp::init( argc, argv );
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
