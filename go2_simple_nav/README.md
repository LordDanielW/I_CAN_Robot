# go2_simple_nav

Lightweight goal-seeking navigation for Unitree Go2 using ROS 2 only (no Nav2, no SLAM). Subscribes to `/odom`, publishes `/cmd_vel`, and relies on the Go2 firmware for obstacle avoidance.

## Usage
Build and source the workspace, then launch:

```bash
colcon build --packages-select go2_simple_nav
source install/setup.bash
ros2 launch go2_simple_nav simple_nav.launch.py goal_x:=3.0 goal_y:=1.5
```

## Parameters
- `goal_x` (float, default 0.0)
- `goal_y` (float, default 0.0)
- `loop_rate` (Hz, default 20.0)
- `max_speed` (m/s, default 0.5)
- `goal_tolerance` (m, default 0.25)
- `goal_topic` (string, default `/goal`) incoming goals as `geometry_msgs/Point`
- `goal_done_topic` (string, default `/goal_reached`) publishes `std_msgs/Bool` when goal reached
- `require_obstacle_on` (bool, default `true`) hold if obstacle avoidance not confirmed
- `obstacle_topic` (string, default `/unitree/obstacle_on`) Bool indicating OA state
- `driver_param_node` (string, default `go2_driver_node`) node to query OA parameter
- `driver_param_name` (string, default `obstacle_avoidance`) parameter to read for OA state
- `cmd_vel_topic` (string, default `/cmd_vel_out`) Twist command output topic

## Quick test: send a goal

After building and sourcing, publish the provided goal once (sent 5 times for reliability):

```bash
colcon build --packages-select go2_simple_nav
source install/setup.bash
ros2 run go2_simple_nav send_nav_goal \
	--ros-args \
	-p goal_topic:=/goal \
	-p x:=1.440033 \
	-p y:=-1.425135 \
	-p z:=0.382595
```

`simple_nav` only uses x/y for planar motion; z is kept for traceability but ignored.

Defaults are also provided in `config/params.yaml`.
