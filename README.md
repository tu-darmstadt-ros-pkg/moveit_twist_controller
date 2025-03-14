# MoveIt Twist Controller

**MoveIt Twist Controller** provides direct 6D end-effector motion via geometry_msgs/TwistStamped commands. 
It leverages the MoveIt inverse kinematics interface and requires a [ros_control](https://control.ros.org/rolling/index.html) *joint position interface*. It also supports collision checks, joint velocity limits, and optional compliance through joint current limits.


## Features

- **Direct End-Effector Control**  
  Send twist commands (`TwistStamped`) to move the end-effector in 6D space.

- **Gripper Control**  
  Send velocity commands (`Float64`) to open/close the gripper.

- **Collision Avoidance**  
  Proposed goal poses are checked for collisions via MoveIt before execution.

- **Joint Velocity Limits**  
  Checks if the requested motion exceeds joint velocity limits (extracted from the moveit::RobotModel).

- **Continuous Joints**  
  If the inverse kinematics solver proposes an angle jump of `n * 2π` for a continuous joint, the jump is ignored.

- **Compliance (Optional)**  
  Enables current-limiting on arm joints (using a dynamically reconfigurable interface) to achieve compliance.


## ROS Interfaces

### Subscribed Topics

1. **`moveit_twist_controller/eef_cmd`** (`geometry_msgs/TwistStamped`)
    - Receives direct velocity commands for the end-effector in 6D space (linear and angular velocities).

2. **`moveit_twist_controller/gripper_cmd`** (`std_msgs/Float64`)
    - Receives velocity commands for the gripper.

### Published Topics

1. **`moveit_twist_controller/goal_pose`** (`geometry_msgs/PoseStamped`)
    - Publishes the current goal pose of the end-effector after IK calculation and collision checks.

2. **`moveit_twist_controller/enabled`** (`std_msgs/Bool`)
    - Indicates whether the controller is currently enabled.

3. **`moveit_twist_controller/robot_state`** (`moveit_msgs/RobotState`)
    - Publishes the goal robot state (with collision checking) for visualization or debugging.

### Services

1. **`moveit_twist_controller/reset_pose`** (`std_srvs/Empty`)
    - Resets the internal goal pose to the current physical pose of the robot.

2. **`moveit_twist_controller/hold_pose`** (`std_srvs/Bool`)
    - When called with `true`, holds the robot end-effector’s world pose while, for instance, moving the robot base.
    - When called with `false`, normal twist-based motions resume.

3. **`moveit_twist_controller/enable_current_limits`** (`std_srvs/Bool`)
    - Enables/disables the per-joint current limit interface to achieve compliance.
    - Requires `request_current_interface` to be enabled in the parameters.


## Parameters

All parameters are typically defined in a YAML file under the namespace `moveit_twist_controller`. Below are the commonly used ones:

| Parameter Name                   | Default Value                               | Description                                                                                                |
|----------------------------------|---------------------------------------------|------------------------------------------------------------------------------------------------------------|
| **`free_angle`**                 | `""`                                        | Axis with a “free” rotation (e.g., for IK in cases of infinite solutions). Acceptable values: `[ "", "x", "y", "z"]`. |
| **`gripper_joint_name`**         | `"gripper_servo_joint"`                     | Name of the gripper joint in the URDF.                                                                     |
| **`group_name`**                 | `"arm_group"`                               | MoveIt group to be controlled (the arm’s MoveIt planning group).                                           |
| **`request_current_interface`**  | `true`                                      | If `true`, the controller will request and use current-limiting interfaces.                                |
| **`current_limits`**             | `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]`       | Per-joint current limits in Amps, used if the current-limiting interface is enabled.                        |
| **`kinematics_solver`**          | `kdl_kinematics_plugin/KDLKinematicsPlugin` | Kinematics solver plugin to use (as an example).                                                        |
| **`kinematics_solver_timeout`**  | `0.05`                                      | Timeout for the IK solver (in seconds).                                                                    |
| **`kinematics_solver_attempts`** |  `3`                                        | Number of attempts allowed for the IK solver.                                                              |
