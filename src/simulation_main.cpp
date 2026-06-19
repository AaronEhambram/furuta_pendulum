#include "FurutaPendulum.hpp"
#include "Visualizer.hpp"
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <thread>

std::atomic<bool> visualizer_running(true);

void run_simulation(FurutaPendulum &pendulum)
{
  double dt = 0.001;
  auto last_time = std::chrono::system_clock::now();
  while (visualizer_running)
  {
    const double motor_torque = 0.0;               // No control input for now
    const double arm_distubance_torque = 0.0;      // No disturbance for now
    const double pendulum_distubance_torque = 0.0; // No disturbance for
    const auto current_time = std::chrono::system_clock::now();
    const std::chrono::duration<double> elapsed_time_to_last_update = current_time - last_time;
    last_time = current_time;
    for (int i = 0; i < elapsed_time_to_last_update.count() / dt; ++i)
    {
      pendulum.update(motor_torque, arm_distubance_torque, pendulum_distubance_torque, dt);
    }
    // sleep for 0.01 seconds to simulate real-time
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

int main()
{
  Parameters params = {0.1, 0.2, 0.2, 0.1, 0.05, 9.81, 0.01, 1e-4, 1e-4, 1e-2, 1e-3};
  FurutaPendulum pendulum(params, {0.0, 0.1, 0.0, 0.0});
  std::thread simulation_thread(run_simulation, std::ref(pendulum));
  Visualizer visualizer(pendulum);
  // Run visualization on main thread (raylib requirement)
  visualizer.showSimulation();
  // Signal simulation thread to stop
  visualizer_running = false;
  simulation_thread.join();
  return 0;
}