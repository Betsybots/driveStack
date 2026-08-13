# MPC Wall-Following Controller

Implemented in [`wall_follower_node.cpp`](../src/wall_follower_node.cpp) as an alternative to
the existing PID controller. Selected at runtime via the `controller_type` parameter
(`"mpc"` default, `"pid"` for the legacy controller). Both read the same left/right/front
distances computed from the filtered, downsampled point cloud.

## Data feeding the controller

Each `pointcloudCallback`:
1. Downsamples the incoming cloud with a PCL voxel grid (`downsampling()`).
2. Filters points to `z_min`..`z_max` to drop ground/ceiling returns.
3. For every surviving point, `computeSectorMins()` updates three nearest-range values by
   angular sector (relative to the sensor's `+x` forward axis):
   - `left_min`: within `side_half_angle_deg` of +90°
   - `right_min`: within `side_half_angle_deg` of -90°
   - `front_min`: within `front_half_angle_deg` of 0°

These three scalars (`left_min`, `right_min`, `front_min`) are the only inputs to the MPC.

## Controller model

`computeMpcCmdVel()` treats wall-following as regulating a virtual corridor cross-track error
using a unicycle model, holding forward speed fixed at `linear_velocity_mps` and optimizing only
the angular velocity `omega`.

State per prediction step `k`:
- `theta_k`: heading change accumulated from the commanded `omega`
- `e_k`: cross-track error, initialized to `e0 = left_min - right_min`
- `x_k`: forward distance traveled, used only for the collision term

Update equations (Euler integration, step `mpc_dt`):
```
theta += omega * dt
e     -= v * sin(theta) * dt
x     += v * cos(theta) * dt
```
Sign convention matches the PID controller: positive `e0` means the robot is closer to the
right wall, and a positive `omega` (turn left) reduces `e`.

## Optimization (control-horizon-1 shooting)

Because no QP/NLP solver dependency is in the workspace, the "optimizer" is a deterministic
grid search:
- `mpc_omega_candidates` candidate values are spread evenly across
  `[-max_angular_velocity_rps, +max_angular_velocity_rps]`.
- Each candidate is held constant for the whole prediction horizon (`mpc_horizon_steps` steps of
  `mpc_dt`) — i.e. control horizon = 1, prediction horizon = N.
- Every candidate is forward-simulated and scored; the lowest-cost candidate's `omega` is applied
  to `cmd.angular.z` this cycle (receding horizon: re-solved from scratch every callback).

## Cost function

Summed over the prediction horizon, per candidate `omega`:

```
cost = Σ_k [ w_lateral * e_k^2 + w_heading * theta_k^2 + w_collision * min(0, margin_k)^2 ]
     + w_control * omega^2
     + w_control_rate * (omega - prev_omega)^2
```
where `margin_k = (front_min - mpc_safety_margin_m) - x_k`, only applied when `front_min` is
finite (i.e. a front obstacle was actually detected). `prev_omega` is the previously applied
command, penalizing large jumps between cycles for smoother steering.

If either `left_min` or `right_min` is not finite (no wall detected on a side), the controller
stops the robot (`linear.x = angular.z = 0`) and resets `prev_mpc_omega_`, matching the PID
controller's fail-safe behavior.

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `controller_type` | `mpc` | `mpc` or `pid` |
| `front_half_angle_deg` | 20.0 | Half-width of the forward detection sector |
| `mpc_horizon_steps` | 15 | Prediction horizon length (N) |
| `mpc_dt` | 0.15 | Seconds per prediction step |
| `mpc_omega_candidates` | 21 | Number of candidate angular velocities searched |
| `mpc_weight_lateral` | 4.0 | Weight on cross-track error `e` |
| `mpc_weight_heading` | 1.0 | Weight on heading error `theta` |
| `mpc_weight_control` | 0.05 | Weight on `omega^2` (effort) |
| `mpc_weight_control_rate` | 0.1 | Weight on change from previous `omega` (smoothing) |
| `mpc_weight_collision` | 50.0 | Weight on predicted front-wall margin violation |
| `mpc_safety_margin_m` | 0.3 | Minimum standoff distance from the detected front wall |

## Known limitations

- Front-collision avoidance only slows the predicted approach (via reduced `cos(theta)`); it does
  not choose a turn direction on its own — direction still comes from the lateral/heading terms.
- `theta` and `x` are reset to 0 each callback (no persisted pose estimate), so the model is a
  short-horizon local approximation, not a full corridor localization.
