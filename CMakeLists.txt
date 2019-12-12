######################
# set up the project #
######################
cmake_minimum_required(VERSION 2.8.3)

project(blmc_robots)

# required to use std::shared_ptr in code and to link the python bindings
if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--no-as-needed")
endif()
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

# ensuring path to libraries are set during install
set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

############################
# Search for dependencies. #
############################

set(CATKIN_PKGS ${CATKIN_PKGS}
  ati_ft_sensor
  blmc_drivers
  master_board_sdk_catkin
  real_time_tools
  mpi_cmake_modules
  robot_interfaces
  pybind11_catkin
  yaml_cpp_catkin
)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PKGS})

find_package(yaml-cpp REQUIRED)
# ${YAML_CPP_INCLUDE_DIR}
# ${YAML_CPP_LIBRARIES}

# search for the robot_properties package for the demos calibration data
find_package(robot_properties_solo QUIET)
find_package(robot_properties_teststand QUIET)
find_package(robot_properties_stuggihop QUIET)

search_for_eigen()

catkin_python_setup()

######################################################
# define the include directory of all ${CATKIN_PKGS} #
######################################################
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    ${Eigen_INCLUDE_DIRS}
)

##########################################
# export the package as a catkin package #
##########################################
set(exported_libraries
  ${PROJECT_NAME}
  test_bench_8_motors
  solo

  teststand
  stuggihop
  single_motor
  single_leg
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${exported_libraries}
  CATKIN_DEPENDS ${CATKIN_PKGS}
  DEPENDS yaml-cpp
)

########################################################
# manage the creation of the libraries and executables #
########################################################
add_subdirectory(src)

#########################
# manage the unit tests #
#########################
add_subdirectory(tests)

####################
# manage the demos #
####################
add_subdirectory(demos)

###################
# python bindings #
###################
add_subdirectory(srcpy)

##########################
# building documentation #
##########################
build_doxygen_documentation()
