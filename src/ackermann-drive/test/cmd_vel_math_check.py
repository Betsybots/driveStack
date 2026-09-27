#!/usr/bin/env python3
"""Standalone re-implementation of ackermann-drive-main.cpp's control math.

No ROS, no phoenix6, no hardware -- this exists purely to let cmd_vel -> output
be inspected on a machine that cannot build/run the real node (no CAN, no
/sys/class/pwm). Every formula below is copied verbatim from the node so the
numbers match what it would actually do; if you change the node, update this.
"""
import math

# ---- parameters, mirrored from config/ackermannDrive.yaml ----
WHEELBASE = 0.31115
MAX_STEER_ANGLE = 0.6981317  # rad, 40 deg
WHEEL_DIAMETER = 0.13335
GEAR_RATIO = 0.5
MAX_LINEAR_VELOCITY = 1.0
MAX_ANGULAR_VELOCITY = 2.0
MIN_SPEED_FOR_STEERING_SOLVE = 0.02

SERVO_MIN_PULSE_NS = 1_000_000
SERVO_CENTER_PULSE_NS = 1_500_000
SERVO_MAX_PULSE_NS = 2_000_000
STEERING_INVERTED = False

MIN_TURNING_RADIUS = WHEELBASE / math.tan(MAX_STEER_ANGLE)


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def cmd_vel_to_state(linear_x_raw, angular_z_raw, prev_steer_angle=0.0):
    """Mirrors cmd_vel_callback(): returns (cmd_body_speed, cmd_steer_angle)."""
    linear_x = clamp(linear_x_raw, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY)
    angular_z = clamp(angular_z_raw, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY)

    if abs(linear_x) >= MIN_SPEED_FOR_STEERING_SOLVE:
        steer_angle = math.atan(WHEELBASE * angular_z / linear_x)
    else:
        steer_angle = prev_steer_angle  # holds last angle, per the node

    steer_angle = clamp(steer_angle, -MAX_STEER_ANGLE, MAX_STEER_ANGLE)
    return linear_x, steer_angle


def steer_angle_to_pulse_ns(cmd_steer_angle):
    """Mirrors apply_steering()'s pulse calculation (ignores slew-rate limiting,
    i.e. this is the target pulse once the modeled servo has caught up)."""
    direction = -1.0 if STEERING_INVERTED else 1.0
    normalized = direction * cmd_steer_angle / MAX_STEER_ANGLE
    if normalized >= 0.0:
        span = SERVO_MAX_PULSE_NS - SERVO_CENTER_PULSE_NS
    else:
        span = SERVO_CENTER_PULSE_NS - SERVO_MIN_PULSE_NS
    pulse = SERVO_CENTER_PULSE_NS + normalized * span
    return int(clamp(pulse, SERVO_MIN_PULSE_NS, SERVO_MAX_PULSE_NS))


def wheel_linear_to_motor_tps(wheel_linear_speed):
    """Mirrors wheel_linear_to_motor_tps()."""
    return wheel_linear_speed / (math.pi * WHEEL_DIAMETER * GEAR_RATIO)


def run_case(label, linear_x, angular_z):
    body_speed, steer_angle = cmd_vel_to_state(linear_x, angular_z)
    pulse_ns = steer_angle_to_pulse_ns(steer_angle)
    motor_tps = wheel_linear_to_motor_tps(body_speed)
    motor_rpm = motor_tps * 60.0
    saturated = abs(math.atan(WHEELBASE * clamp(angular_z, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY)
                              / clamp(linear_x, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY)
                              if abs(linear_x) >= MIN_SPEED_FOR_STEERING_SOLVE else 0.0)) > MAX_STEER_ANGLE + 1e-9

    print(f"{label}")
    print(f"  cmd_vel in       : linear.x={linear_x:+.3f} m/s  angular.z={angular_z:+.3f} rad/s")
    print(f"  body speed (vx)  : {body_speed:+.4f} m/s  (clamp: +/-{MAX_LINEAR_VELOCITY})")
    print(f"  steer angle      : {math.degrees(steer_angle):+7.2f} deg  ({steer_angle:+.4f} rad)"
          f"{'  [CLAMPED to max lock]' if saturated else ''}")
    print(f"  implied radius   : {'inf (straight)' if abs(steer_angle) < 1e-6 else f'{WHEELBASE/math.tan(abs(steer_angle)):.3f} m'}"
          f"  (min drivable: {MIN_TURNING_RADIUS:.3f} m)")
    print(f"  servo pulse      : {pulse_ns} ns  (center={SERVO_CENTER_PULSE_NS}, span={SERVO_MIN_PULSE_NS}-{SERVO_MAX_PULSE_NS})")
    print(f"  motor command    : {motor_tps:+.3f} turns/s = {motor_rpm:+.1f} RPM")
    print()


if __name__ == "__main__":
    print(f"Geometry: wheelbase={WHEELBASE} m, max_steer={math.degrees(MAX_STEER_ANGLE):.1f} deg, "
          f"min_turning_radius={MIN_TURNING_RADIUS:.4f} m\n")

    run_case("1) Straight, half speed",           0.50,  0.00)
    run_case("2) Gentle right turn at cruise",     0.50,  0.60)
    run_case("3) Gentle left turn at cruise",      0.50, -0.60)
    run_case("4) Full-lock turn implied at speed", 0.50,  1.19)   # = wz_max from nav2 config
    run_case("5) Same wz, but crawling slowly",    0.20,  1.19)   # shows saturation risk at low vx
    run_case("6) Reverse, mild turn",             -0.20,  0.30)
    run_case("7) Below steering-solve threshold",  0.01,  0.50)   # angle holds previous value
    run_case("8) Excess angular request (over-lock)", 0.50, 2.00) # angular clamp + atan clamp both engage
