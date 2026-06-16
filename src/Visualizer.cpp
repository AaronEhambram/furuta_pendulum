#include "Visualizer.hpp"
#include <iostream>

Visualizer::Visualizer(FurutaPendulum &pendulum) : pendulum_(pendulum)
{
}

void Visualizer::printState()
{
  State state = pendulum_.getState();
  std::cout << "Arm Angle: " << state.arm_angle << " rad, Pendulum Angle: " << state.pendulum_angle << " rad" << std::endl;
}
