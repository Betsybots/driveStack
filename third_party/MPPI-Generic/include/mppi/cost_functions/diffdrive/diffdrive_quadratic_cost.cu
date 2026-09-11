#include <mppi/cost_functions/diffdrive/diffdrive_quadratic_cost.cuh>

#define RADIUS_SAMPLES 30

DiffdriveQuadraticCost::DiffdriveQuadraticCost(cudaStream_t stream)
{
  bindToStream(stream);
}

float DiffdriveQuadraticCost::computeStateCost(const Eigen::Ref<const output_array> s, int timestep, int* crash_status)
{
    float x = s[0];
    float y = s[1];
    float theta = s[2];

    float state_cost = 0;
    // Calculate Inflation Radius points
    for (int i = 0; i < RADIUS_SAMPLES; i++ )
    {
        float x_r = x + this->params_.inflation_radius * cosf(i * 360/RADIUS_SAMPLES);
        float y_r = y + this->params_.inflation_radius * sinf(i * 360/RADIUS_SAMPLES);

        float y_left_wall = 0;
        for (int j = 0; j <= this->params_.degree; ++j) {
            y_left_wall += this->params_.left_wall_edge_coeffs[j] * std::pow(x_r, j);
        }
        
        float y_right_wall = 0;
        for (int j = 0; j <= this->params_.degree; ++j) {
            y_right_wall += this->params_.right_wall_edge_coeffs[j] * std::pow(x_r, j);
        }

        if ((y_r <= y_right_wall && y_r <= y_left_wall) ||
            (y_r >= y_right_wall && y_r >= y_left_wall))
        return 1e8f;
    }

    float y_left = 0;
    for (int j = 0; j <= this->params_.degree; ++j) {
        y_left += this->params_.left_wall_edge_coeffs[j] * std::pow(x, j);
    }
        
    float y_right = 0;
    for (int j = 0; j <= this->params_.degree; ++j) {
        y_right += this->params_.right_wall_edge_coeffs[j] * std::pow(x, j);
    }

    state_cost += SQ(2 * y - y_right - y_left) * this->params_.robot_position_coeff;
    return state_cost;
}

__device__ float DiffdriveQuadraticCost::computeStateCost(float* state, int timestep, float* theta_c, int* crash_status)
{
    float x = state[0];
    float y = state[1];
    float theta = state[2];
    float state_cost = 0;

    // Calculate Inflation Radius points
    for (int i = 0; i < RADIUS_SAMPLES; i++ )
    {
        float x_r = x + this->params_.inflation_radius * cosf(i * 360/RADIUS_SAMPLES);
        float y_r = y + this->params_.inflation_radius * sinf(i * 360/RADIUS_SAMPLES);

        float y_left_wall = 0;
        for (int j = 0; j <= this->params_.degree; ++j) {
            y_left_wall += this->params_.left_wall_edge_coeffs[j] * std::pow(x_r, j);
        }
        
        float y_right_wall = 0;
        for (int j = 0; j <= this->params_.degree; ++j) {
            y_right_wall += this->params_.right_wall_edge_coeffs[j] * std::pow(x_r, j);
        }

        if ((y_r <= y_right_wall && y_r <= y_left_wall) ||
            (y_r >= y_right_wall && y_r >= y_left_wall))
        return 1e8f;
    }

    float y_left = 0;
    for (int j = 0; j <= this->params_.degree; ++j) {
        y_left += this->params_.left_wall_edge_coeffs[j] * std::pow(x, j);
    }
        
    float y_right = 0;
    for (int j = 0; j <= this->params_.degree; ++j) {
        y_right += this->params_.right_wall_edge_coeffs[j] * std::pow(x, j);
    }

    state_cost += SQ(2 * y - y_right - y_left) * this->params_.robot_position_coeff;
    return state_cost;
}

__device__ float DiffdriveQuadraticCost::terminalCost(float* state, float* theta_c)
{
    float x = state[0];
    float y = state[1];
    float theta = state[2];

    float y_left_wall = 0;
    for (int j = 0; j <= this->params_.degree; ++j) {
        y_left_wall += this->params_.left_wall_edge_coeffs[j] * std::pow(x, j);
    }
        
    float y_right_wall = 0;
    for (int j = 0; j <= this->params_.degree; ++j) {
        y_right_wall += this->params_.right_wall_edge_coeffs[j] * std::pow(x, j);
    }

    if ((y <= y_right_wall && y <= y_left_wall) ||
        (y >= y_right_wall && y >= y_left_wall))
        return 1e8f;

    float terminal_cost = SQ(2 * y - y_right_wall - y_left_wall) * this->params_.robot_position_coeff;
    return terminal_cost;
}

float DiffdriveQuadraticCost::terminalCost(const Eigen::Ref<const output_array> state)
{
    float x = state[0];
    float y = state[1];
    float theta = state[2];

    float y_left_wall = 0;
    for (int j = 0; j <= this->params_.degree; ++j) {
        y_left_wall += this->params_.left_wall_edge_coeffs[j] * std::pow(x, j);
    }
        
    float y_right_wall = 0;
    for (int j = 0; j <= this->params_.degree; ++j) {
        y_right_wall += this->params_.right_wall_edge_coeffs[j] * std::pow(x, j);
    }

    if ((y <= y_right_wall && y <= y_left_wall) ||
        (y >= y_right_wall && y >= y_left_wall))
        return 1e8f;

    float terminal_cost = SQ(2 * y - y_right_wall - y_left_wall) * this->params_.robot_position_coeff;
    return terminal_cost;
}

float DiffdriveQuadraticCost::computeInputCost(const Eigen::Ref<const control_array> u, int timestep, int* crash_status)
{
    float lin_vel_cost = 0.5f * this->params_.robot_lin_vel_coeff * SQ(u[0] - this->params_.max_lin_vel);
    float ang_vel_cost = 0.5f * this->params_.robot_ang_vel_coeff * SQ(u[1] - this->params_.prev_ang_vel);
    this->params_.prev_ang_vel = u[1];
    return lin_vel_cost + ang_vel_cost;
}

__device__ float DiffdriveQuadraticCost::computeInputCost(float* u, int timestep, float* theta_c, int* crash_status)
{
    return 0.0f;
}


