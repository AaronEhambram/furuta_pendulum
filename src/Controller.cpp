#include "Controller.hpp"
#include <cmath>

Controller::Controller(FurutaPendulum &pendulum) : pendulum_(pendulum) {}

void Controller::computeOptimalTorque()
{
  const double &ma = pendulum_.getParameters().arm_mass;
  const double &mp = pendulum_.getParameters().pendulum_mass;
  const double &la = pendulum_.getParameters().arm_length;
  const double &lp = pendulum_.getParameters().pendulum_length;
  const double &g = pendulum_.getParameters().gravity;
  const double &J = pendulum_.getParameters().center_pillar_inertia;
  const double &M = pendulum_.getParameters().pendulum_end_mass;
  const double &b_phi = pendulum_.getParameters().arm_viscouse_friction;
  const double &b_theta = pendulum_.getParameters().pendulum_viscouse_friction;
  const double &tau_c_phi = pendulum_.getParameters().arm_coulomb_friction;
  const double &tau_c_theta = pendulum_.getParameters().pendulum_coulomb_friction;

  const double &phi = pendulum_.getState().arm_angle;
  const double &theta = pendulum_.getState().pendulum_angle;
  const double &phi_dot = pendulum_.getState().arm_angular_velocity;
  const double &theta_dot = pendulum_.getState().pendulum_angular_velocity;

  const double &phi_min = pendulum_.getParameters().arm_angle_min;
  const double &phi_max = pendulum_.getParameters().arm_angle_max;

  // If the pendulum is close to the upright position and not moving much, apply a small torque to kick it up
  if (fabs(theta - M_PI) < 0.01 && fabs(theta_dot) < 0.01)
  {
    torque_mutex_.lock();
    optimal_torque_ = 0.5;
    torque_mutex_.unlock();
    return;
  }

  const double sin_theta = sin(theta);
  const double cos_theta = cos(theta);
  const double sin_sqr_theta = sin_theta * sin_theta;
  const double cos_sqr_theta = cos_theta * cos_theta;

  const double alpha = J + (M + 1.0 / 3.0 * ma + mp) * la * la;
  const double beta = (M + 1.0 / 3.0 * mp) * lp * lp;
  const double gamma = (M + 1.0 / 2.0 * mp) * la * lp;
  const double delta = (M + 1.0 / 2.0 * mp) * g * lp;

  const double Jp = (1.0 / 3.0 * mp + M) * lp * lp;

  const double E = 0.5 * Jp * theta_dot * theta_dot + (0.5 * mp + M) * lp * g * (cos_theta - 1.0);
  const double E_desired = 0.0;
  const double k = 10.0; // Energy gain
  const double energy_error = E - E_desired;
  const double energy_sign =
      (energy_error > 0 && energy_error < 1.0) ? 1.0 : ((energy_error < 0 && energy_error > -1.0) ? -1.0 : 0.0);
  double phi_dot_dot_desired = k * energy_sign * theta_dot * cos_theta;
  const double phi_friction = b_phi * phi_dot + tau_c_phi * (phi_dot > 0 ? 1 : (phi_dot < 0 ? -1 : 0));

  if (phi <= phi_min)
  {
    phi_dot_dot_desired = phi_dot_dot_desired < 0 ? 0 : phi_dot_dot_desired; // Prevent further negative acceleration
  }
  else if (phi >= phi_max)
  {
    phi_dot_dot_desired = phi_dot_dot_desired > 0 ? 0 : phi_dot_dot_desired; // Prevent further positive acceleration
  }

  const double theta_friction = b_theta * theta_dot + tau_c_theta * (theta_dot > 0 ? 1 : (theta_dot < 0 ? -1 : 0));
  const double optimal_torque =
      phi_dot_dot_desired * (alpha + beta * sin_sqr_theta - gamma * gamma / beta * cos_sqr_theta) + phi_friction +
      sin_theta * (cos_theta * (gamma * delta / beta + gamma * cos_theta * phi_dot * phi_dot +
                                2.0 * beta * phi_dot * theta_dot) -
                   gamma * theta_dot * theta_dot) -
      gamma / beta * cos_theta * theta_friction;

  torque_mutex_.lock();
  optimal_torque_ = optimal_torque;
  torque_mutex_.unlock();
}

double Controller::getTorque()
{
  torque_mutex_.lock();
  double torque = optimal_torque_;
  torque_mutex_.unlock();
  return torque;
}
