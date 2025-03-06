#include <moveit_twist_controller/common.hpp>

#include <moveit/robot_state/conversions.hpp>

namespace moveit_twist_controller
{

Eigen::Quaterniond rpyToRot( const Eigen::Vector3d &rpy )
{
  // Create individual angle-axis rotations
  const Eigen::AngleAxisd rollAngle( rpy( 0 ), Eigen::Vector3d::UnitX() );
  const Eigen::AngleAxisd pitchAngle( rpy( 1 ), Eigen::Vector3d::UnitY() );
  const Eigen::AngleAxisd yawAngle( rpy( 2 ), Eigen::Vector3d::UnitZ() );

  // Combine them to form the final rotation
  Eigen::Quaterniond quaternion = yawAngle * pitchAngle * rollAngle;
  return quaternion;
}

} // namespace moveit_twist_controller
