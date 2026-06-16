#include "FurutaPendulum.hpp"
#include <cmath>

FurutaPendulum::FurutaPendulum(const Parameters &params, const State &initial_state) : params_(params), state_(initial_state)
{
}

State FurutaPendulum::getState()
{
  state_mutex_.lock();
  State state_copy = state_;
  state_mutex_.unlock();
  return state_copy;
}

DifferentiatedState FurutaPendulum::modelFunction(const State &state, double torque) const
{
  double ma = params_.arm_mass;
  double mp = params_.pendulum_mass;
  double la = params_.arm_length;
  double lp = params_.pendulum_length;
  double g = params_.gravity;
  double J = params_.center_pillar_inertia;
  double M = params_.pendulum_end_mass;

  double phi = state.arm_angle;
  double theta = state.pendulum_angle;
  double phi_dot = state.arm_angular_velocity;
  double theta_dot = state.pendulum_angular_velocity;

  double sin_theta = sin(theta);
  double cos_theta = cos(theta);
  double sin_sqr_theta = sin_theta * sin_theta;
  double cos_sqr_theta = cos_theta * cos_theta;

  double alpha = J + (M + 1.0 / 3.0 * ma + mp) * la * la;
  double beta = (M + 1.0 / 3.0 * mp) * lp * lp;
  double gamma = (M + 1.0 / 2.0 * mp) * la * lp;
  double delta = (M + 1.0 / 2.0 * mp) * g * lp;

  DifferentiatedState dState;
  dState.arm_angular_velocity = phi_dot;
  dState.pendulum_angular_velocity = theta_dot;
  dState.arm_angular_acceleration = (torque - sin_theta * (cos_theta * (gamma * delta / beta + gamma * cos_theta * phi_dot * phi_dot + 2.0 * beta * phi_dot * theta_dot) - gamma * theta_dot)) / (alpha + beta * sin_sqr_theta - gamma * gamma * cos_sqr_theta);
  dState.pendulum_angular_acceleration = delta / beta * sin_theta + cos_theta * sin_theta * phi_dot * phi_dot - gamma / beta * cos_theta * dState.arm_angular_acceleration;
  return dState;
}

void FurutaPendulum::update(double torque, double dt)
{
  // Runge Kutta 4th order method for numerical integration
  const DifferentiatedState k1 = modelFunction(state_, torque);
  const State state_k2 = {state_.arm_angle + 0.5 * dt * k1.arm_angular_velocity,
                          state_.pendulum_angle + 0.5 * dt * k1.pendulum_angular_velocity,
                          state_.arm_angular_velocity + 0.5 * dt * k1.arm_angular_acceleration,
                          state_.pendulum_angular_velocity + 0.5 * dt * k1.pendulum_angular_acceleration};
  const DifferentiatedState k2 = modelFunction(state_k2, torque);
  const State state_k3 = {state_.arm_angle + 0.5 * dt * k2.arm_angular_velocity,
                          state_.pendulum_angle + 0.5 * dt * k2.pendulum_angular_velocity,
                          state_.arm_angular_velocity + 0.5 * dt * k2.arm_angular_acceleration,
                          state_.pendulum_angular_velocity + 0.5 * dt * k2.pendulum_angular_acceleration};
  const DifferentiatedState k3 = modelFunction(state_k3, torque);
  State state_k4 = {state_.arm_angle + dt * k3.arm_angular_velocity,
                    state_.pendulum_angle + dt * k3.pendulum_angular_velocity,
                    state_.arm_angular_velocity + dt * k3.arm_angular_acceleration,
                    state_.pendulum_angular_velocity + dt * k3.pendulum_angular_acceleration};
  DifferentiatedState k4 = modelFunction(state_k4, torque);

  State state_new = {state_.arm_angle + dt * (k1.arm_angular_velocity + 2.0 * k2.arm_angular_velocity + 2.0 * k3.arm_angular_velocity + k4.arm_angular_velocity) / 6.0,
                     state_.pendulum_angle + dt * (k1.pendulum_angular_velocity + 2.0 * k2.pendulum_angular_velocity + 2.0 * k3.pendulum_angular_velocity + k4.pendulum_angular_velocity) / 6.0,
                     state_.arm_angular_velocity + dt * (k1.arm_angular_acceleration + 2.0 * k2.arm_angular_acceleration + 2.0 * k3.arm_angular_acceleration + k4.arm_angular_acceleration) / 6.0,
                     state_.pendulum_angular_velocity + dt * (k1.pendulum_angular_acceleration + 2.0 * k2.pendulum_angular_acceleration + 2.0 * k3.pendulum_angular_acceleration + k4.pendulum_angular_acceleration) / 6.0};
  state_mutex_.lock();
  state_ = state_new;
  state_mutex_.unlock();
}
