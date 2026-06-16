#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include "FurutaPendulum.hpp"

class Visualizer
{
public:
  Visualizer(FurutaPendulum &pendulum);
  void printState();

private:
  FurutaPendulum &pendulum_;
};

#endif // VISUALIZER_HPP
