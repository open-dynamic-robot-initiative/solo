#
# Copyright (c) 2019, New York University and Max Planck Gesellschaft.
#
# License BSD-3 clause
#

#
# set up the project
#
cmake_minimum_required(VERSION 3.10.2)

project(blmc_robots)

# Using C++17
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

#
# Dependencies
#

# build tools
find_package(ament_cmake REQUIRED)
find_package(mpi_cmake_modules REQUIRED)

# depend on ament macros
find_package(ament_index_cpp REQUIRED)
find_package(ati_ft_sensor REQUIRED)
find_package(blmc_drivers REQUIRED)
find_package(real_time_tools REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(pybind11 REQUIRED)
find_package(yaml_utils REQUIRED)
find_package(Eigen3 REQUIRED)

# export de dependencies
ament_export_dependencies(
  ament_cmake
  mpi_cmake_modules
  ament_index_cpp
  ati_ft_sensor
  blmc_drivers
  real_time_tools
  robot_interfaces
  pybind11
  yaml_utils
  Eigen3)

# prepare the final export
ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

#
# manage the creation of the libraries and executables
#
add_subdirectory(src)

#
# Manage the unit tests.
#
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

#
# Manage the demos.
#
add_subdirectory(demos)

#
# Python bindings.
#
add_subdirectory(srcpy)

#
# Install the package
#
install(DIRECTORY include/ DESTINATION include)

#
# Building documentation.
#
add_documentation()

#
# Export as an ament_package
#
ament_package()
