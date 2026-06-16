#include "FurutaPendulum.hpp"
#include "Visualizer.hpp"
#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <cmath>

void run_simulation(FurutaPendulum &pendulum)
{
  double dt = 0.001;
  auto last_time = std::chrono::system_clock::now();
  const auto simulation_start_time = std::chrono::system_clock::now();
  std::chrono::duration<double> simulation_time_elapsed{};
  while (true)
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
    // Print or log the state as needed
    simulation_time_elapsed = std::chrono::system_clock::now() - simulation_start_time;
    // sleep for 0.01 seconds to simulate real-time
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void run_visualization(Visualizer &visualizer)
{
  while (true)
  {
    visualizer.printState();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

int main()
{
  Parameters params = {0.1, 0.2, 0.2, 0.1, 0.05, 9.81, 0.01};
  FurutaPendulum pendulum(params, {0.0, 0.0, 0.0, 0.0});
  std::thread simulation_thread(run_simulation, std::ref(pendulum));
  Visualizer visualizer(pendulum);
  std::thread visualization_thread(run_visualization, std::ref(visualizer));
  simulation_thread.join();
  visualization_thread.join();

  return 0;
}