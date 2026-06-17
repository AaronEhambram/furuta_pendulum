#include "FurutaPendulum.hpp"
#include "Visualizer.hpp"
#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <cmath>
#include <atomic>

std::atomic<bool> visualizer_running(true);

void run_simulation(FurutaPendulum &pendulum)
{
  double dt = 0.001;
  auto last_time = std::chrono::system_clock::now();
  while (visualizer_running)
  {
    double torque = 0.0; // No control input for now
    auto current_time = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_time_to_last_update = current_time - last_time;
    last_time = current_time;
    for (int i = 0; i < elapsed_time_to_last_update.count() / dt; ++i)
    {
      pendulum.update(torque, dt);
    }
    State state = pendulum.getState();
    // sleep for 0.01 seconds to simulate real-time
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

int main()
{
  Parameters params = {0.1, 0.2, 0.2, 0.1, 0.05, 9.81, 0.01};
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