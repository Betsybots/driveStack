#pragma once

#ifndef DIFFDRIVE_QUADRATIC_COST_CUH_
#define DIFFDRIVE_QUADRATIC_COST_CUH_

#include <mppi/cost_functions/cost.cuh>
#include <mppi/dynamics/diffdrive/diffdrive_dynamics.cuh>
#include <mppi/utils/file_utils.h>

#define INCH_TO_M(x) x * 0.0254;

struct DiffdriveQuadraticCostParams : public CostParams<1>
{
  float robot_position_coeff = 1000;
  float robot_lin_vel_coeff = 100;
  float robot_ang_vel_coeff = 2000;
  float* right_wall_edge_coeffs;
  float* left_wall_edge_coeffs;
  int degree = 3;
  float terminal_cost_coeff = 0;
  float inflation_radius = 3.0;
  float max_lin_vel = 3.0;
  float prev_ang_vel = 0.0;

  DiffdriveQuadraticCostParams()
  {
    this->control_cost_coeff[0] = 10.0;
  }
};

class DiffdriveQuadraticCost : public Cost<DiffdriveQuadraticCost, DiffdriveQuadraticCostParams, DiffdriveDynamicsParams>
{
public:
  /**
   * Constructor
   * @param width
   * @param height
   */
  DiffdriveQuadraticCost(cudaStream_t stream = 0);

  /**
   * @brief Compute the state cost
   */
  __device__ float computeStateCost(float* s, int timestep = 0, float* theta_c = nullptr, int* crash_status = nullptr);

  /**
   * @brief Compute the state cost on the CPU
   */
  float computeStateCost(const Eigen::Ref<const output_array> s, int timestep = 0, int* crash_status = nullptr);

  float computeInputCost(const Eigen::Ref<const control_array> u, int timestep = 0, int* crash_status = nullptr);

  /**
   *
   * @param s current state as a float array
   * @return state cost on GPU
   */
  __device__ float computeInputCost(float* u, int timestep = 0, float* theta_c = nullptr, int* crash_status = nullptr);

  /**
   * @brief Compute the terminal cost of the system
   */
  __device__ float terminalCost(float* s, float* theta_c);

  float terminalCost(const Eigen::Ref<const output_array> s);

  void setWallEdgeCoeffs(float* left_wall_coeffs, float* right_wall_coeffs)
  {
    this->params_.left_wall_edge_coeffs = left_wall_coeffs;
    this->params_.right_wall_edge_coeffs = right_wall_coeffs;
  }

  void setInflationRadius(float in)
  {
    this->params_.inflation_radius = INCH_TO_M(in);
  }

protected:
};

#if __CUDACC__
#include "diffdrive_quadratic_cost.cu"
#endif

#endif  // DIFFDRIVE_QUADRATIC_COST_CUH_// Include the diff drive cost.
