/*
 * Created on Tue Jun 02 2020 by Bogdan Vlahov
 *
 */
#ifndef DIFFDRIVE_DYNAMICS_CUH_
#define DIFFDRIVE_DYNAMICS_CUH_

#include <mppi/dynamics/dynamics.cuh>

struct DiffdriveDynamicsParams : public DynamicsParams
{
  enum class StateIndex : int
  {
    POS_X = 0,
    POS_Y,
    ANG_THETA,
    VEL_X,
    VEL_Y,
    ANG_VEL_THETA,
    NUM_STATES
  };

  enum class ControlIndex : int
  {
    LIN_VEL = 0,
    ANG_VEL,
    NUM_CONTROLS
  };

  enum class OutputIndex : int
  {
    POS_X = 0,
    POS_Y,
    ANG_THETA,
    VEL_X,
    VEL_Y,
    ANG_VEL_THETA,
    NUM_OUTPUTS
  };
  //float tau_roll = 0.25;
  //float tau_pitch = 0.25;
  //float tau_yaw = 0.25;
  float mass = 11.34;  // kg
  DiffdriveDynamicsParams(float mass_in) : mass(mass_in){};
  DiffdriveDynamicsParams() = default;
  ~DiffdriveDynamicsParams() = default;
};

using namespace MPPI_internal;

class DiffdriveDynamics : public Dynamics<DiffdriveDynamics, DiffdriveDynamicsParams>
{
  /**
   * State for this class is defined as follows:
   *    x     - position in 2D space (x, y) - meters
   *    theta - Yaw angle of the Robot
   *
   * Control:
   *    linear velocity  - m/sec
   *    angular velocity - rad/sec
   */
public:
  using PARENT_CLASS = Dynamics<DiffdriveDynamics, DiffdriveDynamicsParams>;

  using state_array = typename PARENT_CLASS::state_array;
  using control_array = typename PARENT_CLASS::control_array;
  using dfdx = typename PARENT_CLASS::dfdx;
  using dfdu = typename PARENT_CLASS::dfdu;

  // Constructor
  DiffdriveDynamics(cudaStream_t stream = 0);
  DiffdriveDynamics(std::array<float2, CONTROL_DIM> control_rngs, cudaStream_t stream = 0);

  using PARENT_CLASS::updateState;  // needed as overloading updateState here hides all parent versions of updateState

  std::string getDynamicsModelName() const override
  {
    return "Diffdrive Model";
  }

  void computeDynamics(const Eigen::Ref<const state_array>& state, const Eigen::Ref<const control_array>& control,
                       Eigen::Ref<state_array> state_der);

  bool computeGrad(const Eigen::Ref<const state_array>& state, const Eigen::Ref<const control_array>& control,
                   Eigen::Ref<dfdx> A, Eigen::Ref<dfdu> B);

  void updateState(const Eigen::Ref<const state_array> state, Eigen::Ref<state_array> next_state,
                   Eigen::Ref<state_array> state_der, const float dt);

  void printState(float* state);

  __device__ void computeDynamics(float* state, float* control, float* state_der, float* theta = nullptr);

  __device__ void updateState(float* state, float* next_state, float* state_der, const float dt);

  state_array stateFromMap(const std::map<std::string, float>& map) override;

  __host__ __device__ void getZeroState(float* state) const;

  state_array getZeroState() const;
};

#if __CUDACC__
#include "diffdrive_dynamics.cu"
#endif
#endif  // DIFFDRIVE_DYNAMICS_CUH_
