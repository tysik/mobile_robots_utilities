cmake_minimum_required(VERSION 2.8)
project(mobile_robots_utilities)

set(CMAKE_CXX_FLAGS "-std=gnu++11 -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors -Wno-write-strings")

include_directories(/usr/include/python2.7)

add_executable(ramps_example examples/ramps_example.cpp ramps.h)
target_link_libraries(ramps_example libpython2.7.so)

add_executable(scaling_example examples/scaling_example.cpp scaling.h)
target_link_libraries(scaling_example libpython2.7.so)

add_executable(continuous_angle_example examples/continuous_angle_example.cpp continuous_angle.h)
target_link_libraries(continuous_angle_example libpython2.7.so)

add_executable(trajectories_example examples/trajectories_example.cpp trajectories.h)
target_link_libraries(trajectories_example libpython2.7.so)
