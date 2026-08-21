#include <mppi/dynamics/diffdrive/diffdrive_dynamics.cuh>
#include <mppi/utils/math_utils.h>

DiffdriveDynamics::DiffdriveDynamics(std::array<float2, CONTROL_DIM> control_rngs, cudaStream_t stream)
  : PARENT_CLASS(control_rngs, stream)
{
  this->params_ = DiffdriveDynamicsParams();
}

DiffdriveDynamics::DiffdriveDynamics(cudaStream_t stream) : PARENT_CLASS(stream)
{
  this->params_ = DiffdriveDynamicsParams();
}

void DiffdriveDynamics::computeDynamics(const Eigen::Ref<const state_array>& state,
                                        const Eigen::Ref<const control_array>& control,
                                        Eigen::Ref<state_array> state_der)
{
  state_der(0) = control[C_INDEX(LIN_VEL)] * cos(state[2]);
  state_der(1) = control[C_INDEX(LIN_VEL)] * sin(state[2]);
  state_der(2) = control[C_INDEX(ANG_VEL)];
}

__device__ void DiffdriveDynamics::computeDynamics(float* state, float* control, float* state_der, float* theta)
{
  float* x = state[0];
  float* y = state[1];
  float* theta = state[2];

  float* vlin = control[0];
  float* vang = control[1];

  float* x_d = state_der[0];
  float* y_d = state_der[1];
  float* theta_d = state_der[2];

  x_d = vlin * cos(theta);
  y_d = vlin * sin(theta);
  theta_d = vang;
}


DiffdriveDynamics::state_array DiffdriveDynamics::stateFromMap(const std::map<std::string, float>& map)
{
  state_array s;
  s(S_INDEX(POS_X)) = map.at("POS_X");
  s(S_INDEX(POS_Y)) = map.at("POS_Y");
  s(S_INDEX(ANG_THETA)) = map.at("ANG_THETA");
  return DiffdriveDynamics::state_array();
}
