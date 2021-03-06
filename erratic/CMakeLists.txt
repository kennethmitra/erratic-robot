cmake_minimum_required(VERSION 3.5)
project(erratic)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(Boost REQUIRED COMPONENTS thread)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})

add_executable(erratic_base src/erratic_base.cpp src/erratic_driver/robot_packets.cpp)
ament_target_dependencies(erratic_base rclcpp geometry_msgs sensor_msgs Boost)
target_link_libraries(erratic_base ${Boost_LIBRARIES})

install(TARGETS
  erratic_base
  DESTINATION lib/${PROJECT_NAME})

ament_package()
