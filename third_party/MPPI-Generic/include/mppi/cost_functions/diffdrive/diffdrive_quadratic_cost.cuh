#pragma once

#ifndef DIFFDRIVE_QUADRATIC_COST_CUH_
#define DIFFDRIVE_QUADRATIC_COST_CUH_

#include <mppi/cost_functions/cost.cuh>
#include <mppi/dynamics/diffdrive/diffdrive_dynamics.cuh>
#include <mppi/utils/file_utils.h>

#define INCH_TO_M(x) x * 0.0254;

struct DiffdriveQuadraticCostParams : public CostParams<1>
{
  // Fixed-size storage (not raw pointers): this struct is copied byte-for-byte to the
  // GPU via cudaMemcpy, so any pointer member would hold a host address and dereferencing
  // it on the device causes an illegal memory access.
  static const int MAX_POLY_DEGREE = 10;

  float robot_position_coeff = 1000;
  float robot_lin_vel_coeff = 100;
  float robot_ang_vel_coeff = 2000;
  float right_wall_edge_coeffs[MAX_POLY_DEGREE + 1] = { 0.0f };
  float left_wall_edge_coeffs[MAX_POLY_DEGREE + 1] = { 0.0f };
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

  // Copies up to MAX_POLY_DEGREE+1 coefficients into the params struct (by value) and
  // pushes the update to the GPU. degree is clamped to what the fixed-size arrays hold.
  void setWallEdgeCoeffs(const float* left_wall_coeffs, const float* right_wall_coeffs, int degree)
  {
    degree = degree < DiffdriveQuadraticCostParams::MAX_POLY_DEGREE
      ? degree : DiffdriveQuadraticCostParams::MAX_POLY_DEGREE;
    this->params_.degree = degree;
    for (int j = 0; j <= degree; ++j) {
      this->params_.left_wall_edge_coeffs[j] = left_wall_coeffs[j];
      this->params_.right_wall_edge_coeffs[j] = right_wall_coeffs[j];
    }
    this->paramsToDevice();
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
