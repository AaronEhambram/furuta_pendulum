#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include "FurutaPendulum.hpp"
#include "raylib.h"

class Visualizer
{
public:
  Visualizer(FurutaPendulum &pendulum);
  void showSimulation();

private:
  FurutaPendulum &pendulum_;
  Camera camera{0};

  // Camera view control variables
  void handleCameraControls();
  // Camera control variables
  float camera_distance_;
  float camera_angle_x_;
  float camera_angle_y_;

  void drawPendulum();

  void drawGrid();
};

#endif // VISUALIZER_HPP
