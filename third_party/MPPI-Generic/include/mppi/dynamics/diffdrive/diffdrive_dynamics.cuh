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
    THETA,
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
    THETA,
    NUM_OUTPUTS
  };

  float max_lin_vel = 10.0;
  float min_lin_vel = 1.0;
  float max_ang_vel = 10.0;
  float min_ang_vel = 1.0;
  float mass = 9.47;  // kg
  
  DiffdriveDynamicsParams() = default;
  DiffdriveDynamicsParams(float max_lin_vel, float min_lin_vel, float max_ang_vel, float min_ang_vel)
    : max_lin_vel(max_lin_vel), min_lin_vel(min_lin_vel), max_ang_vel(max_ang_vel), min_ang_vel(min_ang_vel){};
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
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using PARENT_CLASS = Dynamics<DiffdriveDynamics, DiffdriveDynamicsParams>;

  using state_array = typename PARENT_CLASS::state_array;
  using control_array = typename PARENT_CLASS::control_array;
  using dfdx = typename PARENT_CLASS::dfdx;
  using dfdu = typename PARENT_CLASS::dfdu;

  // Constructor
  DiffdriveDynamics(cudaStream_t stream = 0);
  DiffdriveDynamics(std::array<float2, CONTROL_DIM> control_rngs, cudaStream_t stream = 0);
  DiffdriveDynamics(float max_lin_vel, float min_lin_vel, float max_ang_vel, float min_ang_vel, cudaStream_t stream = 0);

  using PARENT_CLASS::updateState;  // needed as overloading updateState here hides all parent versions of updateState

  std::string getDynamicsModelName() const override
  {
    return "Diffdrive Model";
  }

  /**
   * runs dynamics using state and control and sets it to state
   * derivative. Everything is Eigen Matrices, not Eigen Vectors!
   *
   * @param state     input of current state, passed by reference
   * @param control   input of currrent control, passed by reference
   * @param state_der output of new state derivative, passed by reference
   */
  void computeDynamics(const Eigen::Ref<const state_array>& state, const Eigen::Ref<const control_array>& control,
                       Eigen::Ref<state_array> state_der);

  /**
   * compute the Jacobians with respect to state and control
   *
   * @param state   input of current state, passed by reference
   * @param control input of currrent control, passed by reference

   */
  bool computeGrad(const Eigen::Ref<const state_array>& state, const Eigen::Ref<const control_array>& control,
                   Eigen::Ref<dfdx> A, Eigen::Ref<dfdu> B);

  void printState(const Eigen::Ref<const state_array>& state);
  void printState(float* state);
  void printParams();

  // void computeKinematics(const Eigen::Ref<const state_array>&,
  //                        Eigen::Ref<state_array>& state_der) {};

  __device__ void computeDynamics(float* state, float* control, float* state_der, float* theta = nullptr);

  state_array stateFromMap(const std::map<std::string, float>& map) override;
};

#if __CUDACC__
#include "diffdrive_dynamics.cu"
#endif
#endif  // DIFFDRIVE_DYNAMICS_CUH_
