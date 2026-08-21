#include <mppi/cost_functions/diffdrive/diffdrive_quadratic_cost.cuh>

DiffdriveQuadraticCost::DiffdriveQuadraticCost(cudaStream_t stream)
{
  bindToStream(stream);
}

// different costs
// 1. Quadratic cost based on deviation of desired linear speed from max linear speed
// 2. Quadratic cost based on difference from previous angular speed and current angular speed to reduce jerk
// 3. position of the robot has to be in the center of the walls
// 4. terminal cost for reaching the desired state
// 

/*
float DiffdriveQuadraticCost::computeStateCost(const Eigen::Ref<const output_array> s, int timestep, int* crash_status)
{
  
  return (s[0] - params_.desired_terminal_state[0]) * (s[0] - params_.desired_terminal_state[0]) *
             params_.robot_position_coeff +
         (s[1] - params_.desired_terminal_state[1]) * (s[1] - params_.desired_terminal_state[1]) *
             params_.robot_velocity_coeff +
         (s[2] - params_.desired_terminal_state[2]) * (s[2] - params_.desired_terminal_state[2]) *
             params_.pole_angle_coeff +
         (s[3] - params_.desired_terminal_state[3]) * (s[3] - params_.desired_terminal_state[3]) *
             params_.pole_angular_velocity_coeff;
}
*/
float DiffdriveQuadraticCost::computeStateCost(const Eigen::Ref<const output_array> s, int timestep, int* crash_status)
{
  float quadratic_velocity_cost = ((s[0] - params_.previous_state[0]) * (s[0] - params_.previous_state[0]) +
                             (s[1] - params_.previous_state[1]) * (s[1] - params_.previous_state[1])) * params_.robot_velocity_coeff;
                             
  float quadratic_angular_velocity_cost = ((s[2] - params_.previous_state[2]) * (s[2] - params_.previous_state[2])) * params_.angular_velocity_coeff;
  return quadratic_velocity_cost + quadratic_angular_velocity_cost;
}

__device__ float DiffdriveQuadraticCost::computeStateCost(float* state, int timestep, float* theta_c, int* crash_status)
{
    float quadratic_velocity = (state[0] - params_.previous_state[0]) * (state[0] - params_.previous_state[0]) + \
                               (state[1] - params_.previous_state[1]) * (state[1] - params_.previous_state[1]);
    quadratic_velocity *= params_.robot_velocity_coeff;
    float quadratic_angular_velocity = (state[2] - params_.previous_state[2]) * (state[2] - params_.previous_state[2]);
    quadratic_angular_velocity *= params_.angular_velocity_coeff;
    return quadratic_velocity + quadratic_angular_velocity;
}

__device__ float DiffdriveQuadraticCost::terminalCost(float* state, float* theta_c)
{
  return 0.0;
}
float DiffdriveQuadraticCost::terminalCost(const Eigen::Ref<const output_array> state)
{
  return 0.0;
}
