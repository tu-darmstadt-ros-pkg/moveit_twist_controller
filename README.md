# MoveIt Twist Controller

**MoveIt Twist Controller** provides direct 6D end-effector motion via geometry_msgs/TwistStamped commands. 
It leverages the MoveIt inverse kinematics interface and requires a [ros_control](https://control.ros.org/rolling/index.html) *joint position interface*. It also supports collision checks and joint velocity limits.


## Features

- **Direct End-Effector Control**  
  Send twist commands (`TwistStamped`) to move the end-effector in 6D space.

- **Collision Avoidance**  
  Proposed goal poses are checked for collisions via MoveIt before execution.

- **Joint Velocity Limits**  
  Before sending a motion to the robot, joint velocities (taken from the `moveit::RobotModel`) are verified. If the requested motion would violate these limits, the target pose is scaled back:

  1. Let the current position be $p_c$ and the original target $p_{to}$.
  2. Compute a scaled target $p_t = p_c + \alpha^n (p_{to} - p_c)$ with $0 < \alpha < 1$.
  3. Increase $n$ recursively (up to a maximum number of trials) until no joint-velocity limits are exceeded or the max iteration count is reached.
- **Continuous Joints**  
  If the inverse kinematics solver proposes an angle jump of `n * 2π` for a continuous joint, the jump is ignored.

## ROS Interfaces

### Subscribed Topics

1. **`moveit_twist_controller/eef_cmd`** (`geometry_msgs/TwistStamped`)
    - Receives direct velocity commands for the end-effector in 6D space (linear and angular velocities).

2. **`moveit_twist_controller/nullspace_cmd`** (`std_msgs/Float64MultiArray`)
    - Per-arm-joint bias velocity (rad/s). Biases joint motion while the IK keeps the end-effector pose fixed.

3. **`moveit_twist_controller/joint_cmd`** (`std_msgs/Float64MultiArray`)
    - Per-arm-joint direct jog velocity (rad/s). Drives joints directly without IK; the goal pose is reset on the next eef command.

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

2. **`moveit_twist_controller/hold_mode`** (`std_srvs/SetBool`)
    - When called with `true`, holds the robot end-effector’s world pose while, for instance, moving the robot base.
    - When called with `false`, normal twist-based motions resume.

## Parameters

All parameters are typically defined in a YAML file under the namespace `moveit_twist_controller`. Below are the commonly used ones:

| Parameter Name                                   | Default                                                                                      | Description                                                                                                      |
| ------------------------------------------------ | -------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------- |
| **`group_name`**                                 | `arm_tcp_group`                                                                              | MoveIt planning group to control. Joint names are taken from this group.                                         |
| **`robot_descriptions_loading_timeout`**         | `10.0`                                                                                       | Seconds to wait for the robot descriptions (URDF/SRDF) to load.                                                  |
| **`free_angle`**                                 | `""`                                                                                         | Axis with a “free” rotation (for IK redundancy). Acceptable values: `""`, `"x"`, `"y"`, `"z"`.                   |
| **`velocity_limits`**                            | `[]`                                                                                         | Maximum joint velocities (rad/s) for each arm joint. If empty the limits from the SRDF will be used.             |
| **`velocity_limit_satisfaction_max_iterations`** | `3`                                                                                          | Maximum number of recursive scaling iterations to satisfy joint-velocity limits.                                 |
| **`velocity_limit_satisfaction_multiplicator`**  | `0.33`                                                                                       | Multiplicative factor ($\alpha$) used to scale back target pose when joint-velocity limits are violated.         |

The following parameters are forwarded to the MoveIt group (set on the `robot_description_kinematics.<group_name>` namespace) and can be overridden via the standard MoveIt kinematics YAML:

| Parameter Name                                   | Default                                                                                      | Description                                                                                                      |
| ------------------------------------------------ | -------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------- |
| **`kinematics_solver`**                          | `pick_ik/PickIkPlugin`                                                                       | Plug-in name for the IK solver.                                                                                  |
| **`kinematics_solver_timeout`**                  | `0.05`                                                                                       | Timeout for each IK request (seconds). Make sure controller does not take too long.                              |
| **`kinematics_solver_attempts`**                 | `3`                                                                                          | Number of IK attempts before failing.                                                                            |
| **`solve_type`**                                 | `Distance`                                                                                   | `trac_ik`-specific: strategy for choosing an IK solution. Options: `Distance` (closest to seed) or `Speed` (fastest). |

## Related Controllers

Gripper control and joint/current-limit safety used to live in this package; they now sit in [hector_ros_controllers](https://github.com/tu-darmstadt-ros-pkg/hector_ros_controllers):

- `gripper_position_effort_controller/GripperPositionEffortController` — standalone gripper controller (action + position/velocity topics, optional max-effort limits).
- `safety_position_controller/SafetyPositionController` — chainable position controller that enforces joint and current limits and runs self-collision avoidance. Use it as the downstream controller by setting this package's `chained_controller` parameter to its name.

