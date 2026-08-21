#pragma once

#ifndef DIFFDRIVE_QUADRATIC_COST_CUH_
#define DIFFDRIVE_QUADRATIC_COST_CUH_

#include <mppi/cost_functions/cost.cuh>
#include <mppi/dynamics/diffdrive/diffdrive_dynamics.cuh>
#include <mppi/utils/file_utils.h>

struct DiffdriveQuadraticCostParams : public CostParams<1>
{
  float robot_position_coeff = 1000;
  float robot_velocity_coeff = 100;
  float pole_angle_coeff = 2000;
  float terminal_cost_coeff = 0;
  float desired_terminal_state[4] = { 0, 0, M_PI, 0 };
  float max_linear_speed = 1.0;
  float prev_angular_speed = 0;
  float previous_state[3] = { 0, 0, 0 };
  float linear_speed_coeff = 10.0;
  float angular_speed_coeff = 100.0;

  DiffdriveQuadraticCostParams()
  {
    this->max_linear_speed = 3.0;
    this->control_cost_coeff[0] = 10.0;
    this->linear_speed_coeff = 10.0;
    this->angular_speed_coeff = 10.0;
  }
};

class DiffdriveQuadraticCost : public Cost<DiffdriveQuadraticCost, DiffdriveQuadraticCostParams, DiffdriveDynamicsParams>
{
public:
  /
  /**
   * @brief Constructor
   * @param stream CUDA stream to bind the cost computation to
   */
  __device__ float computeControlCost(float* u, int timestep = 0, int* crash = nullptr);
  /**
   * @brief Destructor
   */
  ~DiffdriveQuadraticCost() = default;
  DiffdriveQuadraticCost(cudaStream_t stream = 0);

  /**
   * @brief Compute the state cost on the CPU
   */
  float computeStateCost(const Eigen::Ref<const output_array> s, int timestep = 0, int* crash_status = nullptr);

  /**
   * @brief Compute the terminal cost of the system
   */
  __device__ float terminalCost(float* s, float* theta_c);

  float terminalCost(const Eigen::Ref<const output_array> s);

protected:
};

#if __CUDACC__
#include "diffdrive_quadratic_cost.cu"
#endif

#endif  // DIFFDRIVE_QUADRATIC_COST_CUH_// Include the diff drive cost.
