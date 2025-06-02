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
  Before sending a motion to the robot, joint velocities (taken from the `moveit::RobotModel`) are verified. If the requested motion would violate these limits, the target pose is scaled back:

  1. Let the current position be $p_c$ and the original target $p_{to}$.
  2. Compute a scaled target $p_t = p_c + \alpha^n (p_{to} - p_c)$ with $0 < \alpha < 1$.
  3. Increase $n$ recursively (up to a maximum number of trials) until no joint-velocity limits are exceeded or the max iteration count is reached.
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

| Parameter Name                                   | Default                                                                                      | Description                                                                                                      |
| ------------------------------------------------ | -------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------- |
| **`group_name`**                                 | `arm_tcp_group`                                                                              | MoveIt planning group to control.                                                                                |
| **`gripper_joint_name`**                         | `gripper_servo_joint`                                                                        | Name of the gripper joint in the URDF.                                                                           |
| **`arm_joints`**                                 | `["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5", "arm_joint_6"]` | List of all joints in the move group (same order).                                                               |
| **`robot_descriptions_loading_timeout`**         | `10.0`                                                                                       | Seconds to wait for the robot descriptions (URDF/SRDF) to load.                                                  |
| **`free_angle`**                                 | `""`                                                                                         | Axis with a “free” rotation (for IK redundancy). Acceptable values: `""`, `"x"`, `"y"`, `"z"`.                   |
| **`velocity_limits`**                            | `[]`                                                                                         | Maximum joint velocities (rad/s) for each arm joint. If empty the limits from the SRDF will be used.             |
| **`velocity_limit_satisfaction_max_iterations`** | `3`                                                                                          | Maximum number of recursive scaling iterations to satisfy joint-velocity limits.                                 |
| **`velocity_limit_satisfaction_multiplicator`**  | `0.33`                                                                                       | Multiplicative factor ($\alpha$) used to scale back target pose when joint-velocity limits are violated.         |
| **`request_current_interface`**                  | `false`                                                                                      | If `true`, the controller will request and use per-joint current-limiting interfaces for compliance.             |
| **`current_limits`**                             | See below, per joint                                                                         | Per-joint current limits (Amps). Contains sub-parameters for each joint:                                         |
|   • `<joint_1>.compliant_limit`                  | `3.0`                                                                                        | Compliant (lower) current limit for `<joint_1>` (Amps).                                                          |
|   • `<arm_joint_1>.stiff_limit`                  | `10.0`                                                                                       | Stiff (higher) current limit for `<joint_1>` (Amps).                                                             |
| **`kinematics_solver`**                          | `trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin`                                          | Plug-in name for the IK solver.                                                                                  |
| **`kinematics_solver_timeout`**                  | `0.001`                                                                                      | Timeout for each IK request (seconds). Make sure controller does not take too long.                              |
| **`kinematics_solver_attempts`**                 | `3`                                                                                          | Number of IK attempts before failing.                                                                            |
| **`solve_type`**                                 | `Distance`                                                                                   | Strategy for choosing an IK solution. Options: `Distance` (closest to seed) or `Speed` (fastest).                |

