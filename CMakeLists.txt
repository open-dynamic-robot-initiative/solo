#
# Copyright (c) 2019, New York University and Max Planck Gesellschaft.
#
# License BSD-3 clause
#

#
# set up the project
#
cmake_minimum_required(VERSION 3.10.2)

project(solo)

# Using C++17
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

OPTION(BUILD_ROS_DYNAMIC_GRAPH "Build with ROS and DynamicGraph" ON)

#
# Dependencies
#

# build tools
find_package(ament_cmake REQUIRED)
find_package(mpi_cmake_modules REQUIRED)

# depend on ament macros.
find_package(odri_control_interface REQUIRED)
find_package(master_board_sdk REQUIRED)
find_package(slider_box REQUIRED)
find_package(real_time_tools REQUIRED)
find_package(pybind11 REQUIRED)
find_package(yaml_utils REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
# Find resources from robot_properties packages.
find_package(PythonModules COMPONENTS robot_properties_solo)

# optional dependencies.
find_package(blmc_drivers QUIET)
find_package(dynamic_graph_manager QUIET)

# Build and export depending on weather blmc_drivers was found or not.
set(BUILD_SOLO8_TI off)
set(export_list
    ament_cmake
    mpi_cmake_modules
    ament_index_cpp
    odri_control_interface
    master_board_sdk
    slider_box
    real_time_tools
    pybind11
    yaml_utils
    Eigen3
    spdlog
)
if(${dynamic_graph_manager_FOUND})
  set(export_list ${export_list} dynamic_graph_manager)
endif()
if(${blmc_drivers_FOUND})
  set(BUILD_SOLO8_TI on)
  set(export_list ${export_list} blmc_drivers)
endif()
ament_export_dependencies(${export_list})

# prepare the final export
ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

#
# manage the creation of the libraries and executables
#
add_subdirectory(src)

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

# Install the include files
install(DIRECTORY include/ DESTINATION include)

# Install python files.
get_python_install_dir(python_install_dir)
install(
  DIRECTORY python/${PROJECT_NAME}
  DESTINATION "${python_install_dir}"
  PATTERN "*.pyc" EXCLUDE
  PATTERN "__pycache__" EXCLUDE)

#
# Building documentation.
#
add_documentation()

#
# Export as an ament_package
#
ament_package()
