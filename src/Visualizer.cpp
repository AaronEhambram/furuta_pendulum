#include "Visualizer.hpp"
#include <iostream>
#include "raylib.h"
#include <cmath>

Visualizer::Visualizer(FurutaPendulum &pendulum) : pendulum_(pendulum)
{
  const int screenWidth = 1600;
  const int screenHeight = 900;

  InitWindow(screenWidth, screenHeight, "Furuta Pendulum Simulation");

  // Define the camera to look into our 3d world
  camera.position = (Vector3){1.0f, 0.0f, 0.0f};
  camera.target = (Vector3){0.0f, 0.0f, 0.0f};
  camera.up = (Vector3){0.0f, 0.0f, 1.0f};
  camera.fovy = 45.0f;
  camera.projection = CAMERA_PERSPECTIVE;
  camera_distance_ = 1.0f;
  camera_angle_y_ = 0.0f;
  camera_angle_x_ = 20.0f;

  SetTargetFPS(60); // Set our game to run at 60 frames-per-second
}

void Visualizer::handleCameraControls()
{
  // Handle mouse movement for camera rotation
  if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
  {
    Vector2 mouseDelta = GetMouseDelta();
    camera_angle_y_ -= mouseDelta.x * 0.2f; // Azimuth (horizontal rotation around Z)
    camera_angle_x_ += mouseDelta.y * 0.2f; // Elevation (vertical rotation)

    // Clamp elevation to prevent flipping over the top
    if (camera_angle_x_ > 89.0f)
      camera_angle_x_ = 89.0f;
    if (camera_angle_x_ < -89.0f)
      camera_angle_x_ = -89.0f;
  }

  // Handle mouse wheel for zoom
  float wheelMove = GetMouseWheelMove();
  camera_distance_ -= wheelMove * 0.2f;
  if (camera_distance_ < 0.2f)
    camera_distance_ = 0.2f; // Minimum zoom
  if (camera_distance_ > 100.0f)
    camera_distance_ = 100.0f; // Maximum zoom

  // Update camera position based on spherical coordinates with Z-up axis
  float elevation = camera_angle_x_ * M_PI / 180.0f;
  float azimuth = camera_angle_y_ * M_PI / 180.0f;
  camera.position.x = camera.target.x + camera_distance_ * cosf(elevation) * cosf(azimuth);
  camera.position.y = camera.target.y + camera_distance_ * cosf(elevation) * sinf(azimuth);
  camera.position.z = camera.target.z + camera_distance_ * sinf(elevation);
}

void Visualizer::drawPendulum()
{
  const Parameters &params = pendulum_.getParameters();
  const State &state = pendulum_.getState();
  const float radius = 0.01f;

  // Draw the base
  DrawCylinderEx((Vector3){0.0f, 0.0f, -0.2f}, (Vector3){0.0f, 0.0f, 0.0f}, radius, radius, 50, Color{100, 100, 100, 255});
  DrawCube((Vector3){0.0f, 0.0f, -0.2f}, 0.05f, 0.05f, 0.05f, Color{150, 150, 150, 255});

  // Draw the arm
  const float arm_end_x = params.arm_length * cosf(state.arm_angle);
  const float arm_end_y = params.arm_length * sinf(state.arm_angle);
  DrawCylinderEx((Vector3){0.0f, 0.0f, 0.0f}, (Vector3){arm_end_x, arm_end_y, 0.0f}, radius, radius, 50, Color{100, 100, 100, 255});

  // Draw the pendulum
  const Vector3 pendulum_start = (Vector3){arm_end_x, arm_end_y, 0.0f};
  const Vector3 pendulum_end = (Vector3){arm_end_x - params.pendulum_length * sinf(state.pendulum_angle) * sinf(state.arm_angle), arm_end_y + params.pendulum_length * cosf(state.arm_angle) * sinf(state.pendulum_angle), params.pendulum_length * cosf(state.pendulum_angle)};
  DrawCylinderEx(pendulum_start, pendulum_end, radius, radius, 50, Color{100, 100, 100, 255});

  // Draw the pendulum mass
  DrawSphereEx(pendulum_end, radius * 2.0f, 50, 50, Color{200, 0, 0, 255});
}

void Visualizer::drawGrid()
{
  const int gridSize = 10;
  const float gridSpacing = 0.05f;
  const float extent = gridSize * gridSpacing / 2.0f;

  for (int i = -gridSize / 2; i <= gridSize / 2; i++)
  {
    float pos = i * gridSpacing;
    DrawLine3D((Vector3){pos, -extent, 0.0f}, (Vector3){pos, extent, 0.0f}, Color{200, 200, 200, 150});
    DrawLine3D((Vector3){-extent, pos, 0.0f}, (Vector3){extent, pos, 0.0f}, Color{200, 200, 200, 150});
  }

  // Draw axes as arrows
  DrawCylinderEx((Vector3){0.0f, 0.0f, 0.0f}, (Vector3){1.7f * gridSpacing, 0.0f, 0.0f}, 0.002f, 0.002f, 50, Color{255, 0, 0, 150});             // X-axis
  DrawCylinderEx((Vector3){1.7f * gridSpacing, 0.0f, 0.0f}, (Vector3){2.0f * gridSpacing, 0.0f, 0.0f}, 0.004f, 0.0f, 50, Color{255, 0, 0, 150}); // X-axis cap
  DrawCylinderEx((Vector3){0.0f, 0.0f, 0.0f}, (Vector3){0.0f, 1.7f * gridSpacing, 0.0f}, 0.002f, 0.002f, 50, Color{0, 255, 0, 150});             // Y-axis
  DrawCylinderEx((Vector3){0.0f, 1.7f * gridSpacing, 0.0f}, (Vector3){0.0f, 2.0f * gridSpacing, 0.0f}, 0.004f, 0.0f, 50, Color{0, 255, 0, 150}); // Y-axis cap
  DrawCylinderEx((Vector3){0.0f, 0.0f, 0.0f}, (Vector3){0.0f, 0.0f, 1.7f * gridSpacing}, 0.002f, 0.002f, 50, Color{0, 0, 255, 150});             // Z-axis
  DrawCylinderEx((Vector3){0.0f, 0.0f, 1.7f * gridSpacing}, (Vector3){0.0f, 0.0f, 2.0f * gridSpacing}, 0.004f, 0.0f, 50, Color{0, 0, 255, 150}); // Z-axis cap
}

void Visualizer::showSimulation()
{
  while (!WindowShouldClose()) // Detect window close button or ESC key
  {
    handleCameraControls();
    BeginDrawing();
    ClearBackground(RAYWHITE);
    BeginMode3D(camera);
    drawPendulum();
    drawGrid();
    EndMode3D();
    DrawFPS(5, 5);
    EndDrawing();
  }

  CloseWindow(); // Close window and OpenGL context
}
