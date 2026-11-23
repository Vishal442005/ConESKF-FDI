cmake_minimum_required(VERSION 3.10)
project(coneskf_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-O3 -march=native -Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(coneskf_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(test_coneskf src/test_coneskf.cpp)
target_include_directories(test_coneskf PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(run_flight_benchmark src/run_flight_benchmark.cpp)
target_include_directories(run_flight_benchmark PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(export_timeseries src/export_timeseries.cpp)
target_include_directories(export_timeseries PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

install(TARGETS test_coneskf run_flight_benchmark export_timeseries
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
