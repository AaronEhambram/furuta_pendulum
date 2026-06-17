#ifndef FURUTA_PENDULUM_HPP
#define FURUTA_PENDULUM_HPP

#include <mutex>

struct State
{
  double arm_angle;
  double pendulum_angle;
  double arm_angular_velocity;
  double pendulum_angular_velocity;
};

struct DifferentiatedState
{
  double arm_angular_velocity;
  double pendulum_angular_velocity;
  double arm_angular_acceleration;
  double pendulum_angular_acceleration;
};

struct Parameters
{
  double arm_length;
  double pendulum_length;
  double arm_mass;
  double pendulum_mass;
  double pendulum_end_mass;
  double gravity;
  double center_pillar_inertia;
};

class FurutaPendulum
{
public:
  FurutaPendulum(const Parameters &params, const State &initial_state = {0.0, 0.0, 0.0, 0.0});

  void update(double torque, double dt);
  DifferentiatedState modelFunction(const State &state, double torque) const;
  State getState(); // thread-safe getter for the current state of the system
  Parameters getParameters() const { return params_; }

private:
  Parameters params_;
  State state_;
  std::mutex state_mutex_; // Mutex to protect access to the state
};

#endif // FURUTA_PENDULUM_HPP
