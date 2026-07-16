# Drive Stack

Drive Stack is a ROS 2 Jazzy package for controlling a differential-drive mobile robot using a Logitech F710 wireless gamepad.

The stack runs on a Raspberry Pi 5 with Ubuntu 24.04 and converts joystick inputs into ROS 2 velocity commands. It also provides differential-drive calculations for converting linear and angular velocity commands into individual left- and right-wheel motor commands.

## Main Features

* Logitech F710 joystick integration
* Manual robot teleoperation
* Publishes velocity commands using `geometry_msgs/msg/Twist`
* Differential-drive kinematics
* Left- and right-wheel velocity calculation
* Configurable maximum linear and angular speeds
* Joystick dead-zone handling
* Emergency-stop and enable-button support
* Designed for Raspberry Pi 5
* Compatible with Ubuntu 24.04 and ROS 2 Jazzy

## System Flow

```text
Logitech F710 Gamepad
          │
          ▼
ROS 2 joy_node
          │
          ▼
Joystick Teleoperation Node
          │
          ▼
        /cmd_vel
          │
          ▼
Differential-Drive Controller
          │
          ├── Left Motor Command
          └── Right Motor Command
```

## Target Platform

* Raspberry Pi 5
* Ubuntu 24.04
* ROS 2 Jazzy
* Logitech F710 wireless gamepad
* Differential-drive mobile robot
