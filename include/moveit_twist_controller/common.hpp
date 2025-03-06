#ifndef MOVEIT_JOYSTICK_CONTROL_COMMON_H
#define MOVEIT_JOYSTICK_CONTROL_COMMON_H

#include <Eigen/Eigen>

namespace moveit_twist_controller
{

Eigen::Quaterniond rpyToRot( const Eigen::Vector3d &rpy );

} // namespace moveit_twist_controller

#endif // MOVEIT_JOYSTICK_CONTROL_COMMON_H
