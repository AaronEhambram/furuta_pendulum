#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "FurutaPendulum.hpp"
#include <mutex>

class Controller
{
public:
  Controller(FurutaPendulum &pendulum);
  void computeOptimalTorque();
  double getTorque();

private:
  FurutaPendulum &pendulum_;
  double optimal_torque_;
  std::mutex torque_mutex_;
};

#endif // CONTROLLER_HPP
