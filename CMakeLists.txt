cmake_minimum_required(VERSION 3.10)
project(furuta_pendulum)

find_package (Threads REQUIRED)
find_package(raylib 3.0 REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

include_directories(src) 
add_executable(furuta_pendulum src/simulation_main.cpp src/FurutaPendulum.cpp src/Visualizer.cpp) 
target_link_libraries(furuta_pendulum ${CMAKE_THREAD_LIBS_INIT} raylib)