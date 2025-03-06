#include <moveit_twist_controller/common.hpp>

#include <kdl/frames.hpp>
#include <rclcpp/rclcpp.hpp>

#include <moveit/robot_state/conversions.hpp>

namespace moveit_twist_controller
{

Eigen::Quaterniond rpyToRot( const Eigen::Vector3d &rpy )
{
  const KDL::Rotation kdl_rot = KDL::Rotation::RPY( rpy( 0 ), rpy( 1 ), rpy( 2 ) );
  Eigen::Quaterniond quat;
  kdl_rot.GetQuaternion( quat.x(), quat.y(), quat.z(), quat.w() );
  return quat;
}

Eigen::Vector3d rotToRpy( const Eigen::Matrix3d &rot )
{
  Eigen::Quaterniond rot_quad( rot );
  const KDL::Rotation kdl_rot =
      KDL::Rotation::Quaternion( rot_quad.x(), rot_quad.y(), rot_quad.z(), rot_quad.w() );
  double roll, pitch, yaw;
  kdl_rot.GetRPY( roll, pitch, yaw );
  return { roll, pitch, yaw };
}

/*std::vector<double> stateFromList(const sensor_msgs::msg::JointState& last_state, const std::vector<std::string>& joint_names) {
  std::vector<double> state(joint_names.size(), 0);
  for (unsigned int name_idx = 0; name_idx < joint_names.size(); name_idx++) {
    auto it = std::find(last_state.name.begin(), last_state.name.end(), joint_names[name_idx]);
    if (it != last_state.name.end()) {
      const unsigned int state_idx = it - last_state.name.begin();
      state[name_idx] = last_state.position[state_idx];
    } else {
      RCLCPP_WARN( rclcpp::get_logger("moveit_twist_controller"), "Joint '%s' wasn't found in received joint state.", joint_names[name_idx].c_str());
    }
  }
  return state;
}*/

void updateRobotStatePose( moveit::core::RobotState &state, const Eigen::Affine3d &pose )
{
  sensor_msgs::msg::JointState virtual_link_joint_states;
  virtual_link_joint_states.name.emplace_back( "world_virtual_joint/trans_x" );
  virtual_link_joint_states.name.emplace_back( "world_virtual_joint/trans_y" );
  virtual_link_joint_states.name.emplace_back( "world_virtual_joint/trans_z" );
  virtual_link_joint_states.name.emplace_back( "world_virtual_joint/rot_x" );
  virtual_link_joint_states.name.emplace_back( "world_virtual_joint/rot_y" );
  virtual_link_joint_states.name.emplace_back( "world_virtual_joint/rot_z" );
  virtual_link_joint_states.name.emplace_back( "world_virtual_joint/rot_w" );

  virtual_link_joint_states.position.resize( 7 );
  virtual_link_joint_states.position[0] = pose.translation().x();
  virtual_link_joint_states.position[1] = pose.translation().y();
  virtual_link_joint_states.position[2] = pose.translation().z();
  Eigen::Quaterniond quat( pose.linear() );
  virtual_link_joint_states.position[3] = quat.x();
  virtual_link_joint_states.position[4] = quat.y();
  virtual_link_joint_states.position[5] = quat.z();
  virtual_link_joint_states.position[6] = quat.w();
  moveit::core::jointStateToRobotState( virtual_link_joint_states, state );
}

} // namespace moveit_twist_controller
