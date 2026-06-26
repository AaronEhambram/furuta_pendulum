#include "FurutaPendulum.hpp"
#include <cmath>

FurutaPendulum::FurutaPendulum(const Parameters &params, const State &initial_state)
    : params_(params), state_(initial_state)
{
}

State FurutaPendulum::getState()
{
  state_mutex_.lock();
  State state_copy = state_;
  state_mutex_.unlock();
  return state_copy;
}

DifferentiatedState FurutaPendulum::modelFunction(const State &state, const double &motor_torque,
                                                  const double &arm_distubance_torque,
                                                  const double &pendulum_distubance_torque) const
{
  const double &ma = params_.arm_mass;
  const double &mp = params_.pendulum_mass;
  const double &la = params_.arm_length;
  const double &lp = params_.pendulum_length;
  const double &g = params_.gravity;
  const double &J = params_.center_pillar_inertia;
  const double &M = params_.pendulum_end_mass;

  const double &phi = state.arm_angle;
  const double &theta = state.pendulum_angle;
  const double &phi_dot = state.arm_angular_velocity;
  const double &theta_dot = state.pendulum_angular_velocity;

  const double sin_theta = sin(theta);
  const double cos_theta = cos(theta);
  const double sin_sqr_theta = sin_theta * sin_theta;
  const double cos_sqr_theta = cos_theta * cos_theta;

  const double alpha = J + (M + 1.0 / 3.0 * ma + mp) * la * la;
  const double beta = (M + 1.0 / 3.0 * mp) * lp * lp;
  const double gamma = (M + 1.0 / 2.0 * mp) * la * lp;
  const double delta = (M + 1.0 / 2.0 * mp) * g * lp;

  // Determine torque at the angle limits
  const double K_stop = params_.arm_stop_spring_stiffness;
  const double D_stop = params_.arm_stop_spring_damping;
  double tau_arm_limit = 0.0;
  if (phi < params_.arm_angle_min)
  {
    tau_arm_limit = -K_stop * (phi - params_.arm_angle_min) - D_stop * phi_dot;
  }
  else if (phi > params_.arm_angle_max)
  {
    tau_arm_limit = -K_stop * (phi - params_.arm_angle_max) - D_stop * phi_dot;
  }

  const double tau_arm = motor_torque - params_.arm_viscouse_friction * phi_dot -
                         params_.arm_coulomb_friction * (phi_dot > 0 ? 1 : (phi_dot < 0 ? -1 : 0)) +
                         arm_distubance_torque + tau_arm_limit;
  const double tau_pendulum = -params_.pendulum_viscouse_friction * theta_dot -
                              params_.pendulum_coulomb_friction * (theta_dot > 0 ? 1 : (theta_dot < 0 ? -1 : 0)) +
                              pendulum_distubance_torque;

  DifferentiatedState dState;
  dState.arm_angular_velocity = phi_dot;
  dState.pendulum_angular_velocity = theta_dot;
  dState.arm_angular_acceleration =
      (tau_arm - gamma / beta * cos_theta * tau_pendulum -
       sin_theta * (cos_theta * (gamma * delta / beta + gamma * cos_theta * phi_dot * phi_dot +
                                 2.0 * beta * phi_dot * theta_dot) -
                    gamma * theta_dot * theta_dot)) /
      (alpha + beta * sin_sqr_theta - gamma * gamma / beta * cos_sqr_theta);
  dState.pendulum_angular_acceleration = delta / beta * sin_theta + cos_theta * sin_theta * phi_dot * phi_dot -
                                         gamma / beta * cos_theta * dState.arm_angular_acceleration +
                                         tau_pendulum / beta;
  return dState;
}

void FurutaPendulum::update(const double &motor_torque, const double &arm_distubance_torque,
                            const double &pendulum_distubance_torque, const double &dt)
{
  // Runge Kutta 4th order method for numerical integration
  const DifferentiatedState k1 = modelFunction(state_, motor_torque, arm_distubance_torque, pendulum_distubance_torque);
  const State state_k2 = {state_.arm_angle + 0.5 * dt * k1.arm_angular_velocity,
                          state_.pendulum_angle + 0.5 * dt * k1.pendulum_angular_velocity,
                          state_.arm_angular_velocity + 0.5 * dt * k1.arm_angular_acceleration,
                          state_.pendulum_angular_velocity + 0.5 * dt * k1.pendulum_angular_acceleration};
  const DifferentiatedState k2 =
      modelFunction(state_k2, motor_torque, arm_distubance_torque, pendulum_distubance_torque);
  const State state_k3 = {state_.arm_angle + 0.5 * dt * k2.arm_angular_velocity,
                          state_.pendulum_angle + 0.5 * dt * k2.pendulum_angular_velocity,
                          state_.arm_angular_velocity + 0.5 * dt * k2.arm_angular_acceleration,
                          state_.pendulum_angular_velocity + 0.5 * dt * k2.pendulum_angular_acceleration};
  const DifferentiatedState k3 =
      modelFunction(state_k3, motor_torque, arm_distubance_torque, pendulum_distubance_torque);
  State state_k4 = {state_.arm_angle + dt * k3.arm_angular_velocity,
                    state_.pendulum_angle + dt * k3.pendulum_angular_velocity,
                    state_.arm_angular_velocity + dt * k3.arm_angular_acceleration,
                    state_.pendulum_angular_velocity + dt * k3.pendulum_angular_acceleration};
  DifferentiatedState k4 = modelFunction(state_k4, motor_torque, arm_distubance_torque, pendulum_distubance_torque);

  State state_new = {state_.arm_angle + dt *
                                            (k1.arm_angular_velocity + 2.0 * k2.arm_angular_velocity +
                                             2.0 * k3.arm_angular_velocity + k4.arm_angular_velocity) /
                                            6.0,
                     state_.pendulum_angle + dt *
                                                 (k1.pendulum_angular_velocity + 2.0 * k2.pendulum_angular_velocity +
                                                  2.0 * k3.pendulum_angular_velocity + k4.pendulum_angular_velocity) /
                                                 6.0,
                     state_.arm_angular_velocity +
                         dt *
                             (k1.arm_angular_acceleration + 2.0 * k2.arm_angular_acceleration +
                              2.0 * k3.arm_angular_acceleration + k4.arm_angular_acceleration) /
                             6.0,
                     state_.pendulum_angular_velocity +
                         dt *
                             (k1.pendulum_angular_acceleration + 2.0 * k2.pendulum_angular_acceleration +
                              2.0 * k3.pendulum_angular_acceleration + k4.pendulum_angular_acceleration) /
                             6.0};
  state_mutex_.lock();
  state_ = state_new;
  state_mutex_.unlock();
}
