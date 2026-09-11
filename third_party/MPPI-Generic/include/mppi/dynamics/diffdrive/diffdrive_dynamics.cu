#include <mppi/dynamics/diffdrive/diffdrive_dynamics.cuh>
#include <mppi/utils/math_utils.h>

DiffdriveDynamics::DiffdriveDynamics(cudaStream_t stream)
  : Dynamics<DiffdriveDynamics, DiffdriveDynamicsParams>(stream)
{
}

DiffdriveDynamics::DiffdriveDynamics(float max_lin_vel, float min_lin_vel, float max_ang_vel, float min_ang_vel, cudaStream_t stream)
  : Dynamics<DiffdriveDynamics, DiffdriveDynamicsParams>(stream)
{
  this->params_ = DiffdriveDynamicsParams(max_lin_vel, min_lin_vel, max_ang_vel, min_ang_vel);
}

bool DiffdriveDynamics::computeGrad(const Eigen::Ref<const state_array>& state,
                                   const Eigen::Ref<const control_array>& control, Eigen::Ref<dfdx> A,
                                   Eigen::Ref<dfdu> B)
{
  float theta = state(2);
  float lin_vel = ((control(0) < 0)? -1 : 1)*
                   std::clamp(std::abs(control(0)), this->params_.min_lin_vel, this->params_.max_lin_vel);
  float sin_theta = sinf(theta);
  float cos_theta = cosf(theta);

  A(0, 2) = -lin_vel * sin_theta;
  A(1, 2) = lin_vel * cos_theta;
  
  B(0, 0) = cos_theta;
  B(1, 0) = sin_theta;
  B(2, 1) = 1.0;
  return true;
}

void DiffdriveDynamics::computeDynamics(const Eigen::Ref<const state_array>& state,
                                        const Eigen::Ref<const control_array>& control,
                                        Eigen::Ref<state_array> state_der)
{
  float theta = state(S_INDEX(THETA));
  float lin_vel = ((control(C_INDEX(LIN_VEL)) < 0)? -1 : 1)*
                   std::clamp(std::abs(control(C_INDEX(LIN_VEL))), this->params_.min_lin_vel, this->params_.max_lin_vel);
  float ang_vel = ((control(C_INDEX(ANG_VEL)) < 0)? -1 : 1)*
                    std::clamp(std::abs(control(C_INDEX(ANG_VEL))), this->params_.min_ang_vel, this->params_.max_ang_vel);
  const float sin_theta = sinf(theta);
  const float cos_theta = cosf(theta);

  state_der(S_INDEX(POS_X)) = lin_vel * cos_theta;
  state_der(S_INDEX(POS_Y)) = lin_vel * sin_theta;
  state_der(S_INDEX(THETA)) = ang_vel;
}

void DiffdriveDynamics::printState(const Eigen::Ref<const state_array>& state)
{
  printf("Robot position: [%f, %f]; Robot Angle: %f\n", state(0), state(1), state(2));  // Needs to be completed
}

void DiffdriveDynamics::printState(float* state)
{
  printf("Robot position: [%f, %f]; Robot Angle: %f\n", state[0], state[1], state[2]);  // Needs to be completed
}


__device__ void DiffdriveDynamics::computeDynamics(float* state, float* control, float* state_der, float* theta_s)
{
  float theta = state[S_INDEX(THETA)];
  // std::clamp is host-only constexpr; use fminf/fmaxf so this compiles for the GPU.
  float lin_vel = ((control[C_INDEX(LIN_VEL)] < 0)? -1 : 1)*
                   fminf(fmaxf(fabsf(control[C_INDEX(LIN_VEL)]), this->params_.min_lin_vel), this->params_.max_lin_vel);
  float ang_vel = ((control[C_INDEX(ANG_VEL)] < 0)? -1 : 1)*
                    fminf(fmaxf(fabsf(control[C_INDEX(ANG_VEL)]), this->params_.min_ang_vel), this->params_.max_ang_vel);
  const float sin_theta = sinf(theta);
  const float cos_theta = cosf(theta);

  state_der[S_INDEX(POS_X)] = lin_vel * cos_theta;
  state_der[S_INDEX(POS_Y)] = lin_vel * sin_theta;
  state_der[S_INDEX(THETA)] = ang_vel;
}

Dynamics<DiffdriveDynamics, DiffdriveDynamicsParams>::state_array
DiffdriveDynamics::stateFromMap(const std::map<std::string, float>& map)
{
  state_array s;
  s(S_INDEX(POS_X)) = map.at("POS_X");
  s(S_INDEX(POS_Y)) = map.at("POS_Y");
  s(S_INDEX(THETA)) = map.at("THETA");
  return s;
}
