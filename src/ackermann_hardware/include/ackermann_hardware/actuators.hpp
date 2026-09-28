#ifndef ACKERMANN_HARDWARE__ACTUATORS_HPP_
#define ACKERMANN_HARDWARE__ACTUATORS_HPP_

// Thin actuator interfaces so the ros2_control system can run against the real
// Kraken X60 / PWM servo, or against in-process mocks for bench and CI testing.

namespace ackermann_hardware
{

class DriveMotor
{
public:
  virtual ~DriveMotor() = default;

  // Closed-loop velocity request in motor turns per second.
  virtual void set_velocity_tps(double motor_tps) = 0;
  // Release the motor to its configured neutral mode.
  virtual void stop() = 0;
  // Keeps the motor enabled for the next few control cycles (Phoenix 6 non-FRC
  // enable). If the control loop stalls, the motor disables itself.
  virtual void feed_enable() {}
  // Advances simulated state; the real motor ignores it.
  virtual void tick(double /*dt*/) {}

  // Raw rotor position in turns, as reported by the motor (may wrap).
  virtual double position_turns() = 0;
  virtual double velocity_tps() = 0;
};

class SteeringServo
{
public:
  virtual ~SteeringServo() = default;
  virtual void set_pulse_ns(long pulse_ns) = 0;
};

class MockDriveMotor : public DriveMotor
{
public:
  void set_velocity_tps(double motor_tps) override {command_tps_ = motor_tps;}
  void stop() override {command_tps_ = 0.0;}
  void tick(double dt) override
  {
    velocity_tps_ = command_tps_;
    position_turns_ += velocity_tps_ * dt;
  }
  double position_turns() override {return position_turns_;}
  double velocity_tps() override {return velocity_tps_;}

private:
  double command_tps_ = 0.0;
  double velocity_tps_ = 0.0;
  double position_turns_ = 0.0;
};

class MockSteeringServo : public SteeringServo
{
public:
  void set_pulse_ns(long pulse_ns) override {last_pulse_ns_ = pulse_ns;}
  long last_pulse_ns() const {return last_pulse_ns_;}

private:
  long last_pulse_ns_ = 0;
};

}  // namespace ackermann_hardware

#endif  // ACKERMANN_HARDWARE__ACTUATORS_HPP_
