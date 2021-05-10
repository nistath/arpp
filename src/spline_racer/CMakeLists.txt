cmake_minimum_required(VERSION 3.5)
project(spline_racer)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Werror -Wall -Wextra -Wpedantic -Wno-sign-compare)
endif()

include_directories(include/spline_racer)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interface_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

add_executable(main src/main.cpp src/Track.cpp)

ament_target_dependencies(main rclcpp interface_msgs std_msgs Eigen3)

install(TARGETS
  main
  DESTINATION lib/${PROJECT_NAME})

ament_package()
