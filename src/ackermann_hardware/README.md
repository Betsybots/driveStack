# ackermann_hardware

ros2_control hardware interface (ROS 2 Humble) for a rear-drive, front-steer Ackermann robot:

* **Drive:** one Kraken X60 (TalonFX, Phoenix 6 C++) on the solid rear axle, running closed-loop velocity on the motor controller.
* **Steering:** AGFRC A73BHLW v2 servo on Raspberry Pi hardware PWM, GPIO18 (`pwmchip0/pwm0`).

The plugin exports one traction joint and one steering joint. It is meant to run with
`bicycle_steering_controller` from `ros2_controllers`, which handles the Twist to wheel speed and
steering angle kinematics, the command timeout, and odometry.

| Joint | Command | State |
|---|---|---|
| `rear_wheel_joint` (traction) | `velocity` (wheel rad/s) | `position` (wheel rad), `velocity` (wheel rad/s) |
| `steering_joint` | `position` (rad, + = left) | `position` (rad, open-loop estimate) |

The servo has no feedback, so the reported steering angle is the commanded angle rate-limited
by `steering_slew_rate`.

## Safety behaviour

* The TalonFX stays enabled only while `write()` keeps calling Phoenix `FeedEnable(100 ms)`. If
  the controller manager stalls or dies, the motor disables itself.
* On activate and deactivate the motor goes to neutral and the servo is centred.
* A NaN command stops the wheel and holds the last steering angle. A controller writes NaN when
  it has no reference.
* Commands are clamped to `max_wheel_velocity` and `max_steer_angle`.
* `bicycle_steering_controller` zeroes its reference after `reference_timeout` (0.5 s).

## Parameters

Hardware parameters are the `<param>` tags inside `<hardware>` in
[`urdf/ackermann.ros2_control.xacro`](urdf/ackermann.ros2_control.xacro). They cover:

* the CAN bus and motor ID
* the gear ratio
* current limits
* velocity gains
* PWM chip and channel
* servo pulse widths
* steering direction
* maximum steering angle

Robot geometry (`wheelbase`, wheel radius) lives in
[`config/ackermann_controllers.yaml`](config/ackermann_controllers.yaml). Measure it on the robot.

Set `use_mock_hardware` to `true` to run without CAN or PWM. Commands are then echoed back as
states, which is useful for testing the controller chain on a laptop.

## Raspberry Pi setup

1. Enable hardware PWM on GPIO18: add `dtoverlay=pwm` to `/boot/firmware/config.txt` (or
   `/boot/config.txt`), reboot, then check that `ls /sys/class/pwm/pwmchip0` works. The user
   running `ros2_control_node` needs write access to `/sys/class/pwm`.
2. Bring up the CAN bus, for example with `socketcan_start.sh` at the repo root.
3. Install Phoenix 6 for C++. If it is missing, the package still builds but refuses to start
   unless `use_mock_hardware=true`.

## Calibrating the servo

1. With the robot up on blocks, launch with the joystick.
2. Adjust `servo_center_pulse_ns` until the wheels point straight.
3. Adjust `servo_min_pulse_ns` and `servo_max_pulse_ns` until full lock is reached without
   the servo stalling.
4. Set `max_steer_angle` to the measured angle at full lock.
5. If a positive (left) command turns the wheels right, set `steering_inverted` to `true`.

## Run

```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro \
  ros-humble-joy ros-humble-teleop-twist-joy
colcon build --packages-select ackermann_hardware
source install/setup.bash

# Drive with the Logitech F710 (hold RB)
ros2 launch ackermann_hardware ackermann_control.launch.py use_joystick:=true

# Same thing without hardware
ros2 launch ackermann_hardware ackermann_control.launch.py use_mock_hardware:=true use_joystick:=true
```

The controller takes `geometry_msgs/Twist` on `/bicycle_steering_controller/reference_unstamped`.
To drive from Nav2, remap its `cmd_vel` output there. Odometry is published on
`/bicycle_steering_controller/odometry` and joint states on `/joint_states`.

## Tests

```bash
colcon test --packages-select ackermann_hardware && colcon test-result --verbose
```

* `test_ackermann_config` covers parameter parsing, the pulse mapping, slew, unit conversions and
  encoder wrap.
* `test_ackermann_system` loads the plugin through pluginlib in mock mode. It checks the exported
  interfaces, driving and steering, clamping, NaN handling, and rejection of bad parameters.
