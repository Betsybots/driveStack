# challenge_bringup

Bootstrap bringup package for DIY Challenge 2026.

## Launch

Run the SAD-aligned starter launch:

```bash
ros2 launch challenge_bringup challenge_master.launch.py
```

Useful toggles:

```bash
ros2 launch challenge_bringup challenge_master.launch.py use_nav2:=true use_joystick:=true use_motor_driver:=true
```

## Current Wiring

- Joystick teleop output: `/cmd_vel_joy`
- Collision monitor output target: `/cmd_vel_safe`
- Motor driver input remap: `/cmd_vel_safe` -> `differential-drive`
- LiDAR obstacle source for Nav2: `/hesai/points`

## Pending (Next Step)

- Add `diy_cmd_vel_mux` to combine `/cmd_vel_joy` and `/cmd_vel_nav`
- Add E-stop lockout path from STM32
- Add RTK + EKF stack from SAD
