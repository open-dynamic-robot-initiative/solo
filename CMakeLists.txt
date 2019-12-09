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

set(py_blmc_single_leg_SRC_FILES 
  src/single_leg.cpp
  srcpy/py_single_leg.cpp
)
pybind11_add_module(blmc_single_leg ${py_blmc_single_leg_SRC_FILES})
target_link_libraries(blmc_single_leg PRIVATE single_leg)
set_target_properties(blmc_single_leg PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
install(TARGETS blmc_single_leg DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})


pybind11_add_module(py_real_finger srcpy/py_real_finger.cpp)
target_link_libraries(py_real_finger PRIVATE ${catkin_LIBRARIES} ${PROJECT_NAME} ${YAML_CPP_LIBRARIES})
set_target_properties(py_real_finger PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
install(TARGETS py_real_finger DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})


pybind11_add_module(py_trifinger srcpy/py_trifinger.cpp)
target_link_libraries(py_trifinger PRIVATE ${catkin_LIBRARIES} ${PROJECT_NAME} ${YAML_CPP_LIBRARIES})
set_target_properties(py_trifinger PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
install(TARGETS py_trifinger DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})


pybind11_add_module(py_one_joint srcpy/py_one_joint.cpp)
target_link_libraries(py_one_joint PRIVATE ${catkin_LIBRARIES} ${PROJECT_NAME} ${YAML_CPP_LIBRARIES})
set_target_properties(py_one_joint PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
install(TARGETS py_one_joint DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})


pybind11_add_module(py_two_joint srcpy/py_two_joint.cpp)
target_link_libraries(py_two_joint PRIVATE ${catkin_LIBRARIES} ${PROJECT_NAME} ${YAML_CPP_LIBRARIES})
set_target_properties(py_two_joint PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
install(TARGETS py_two_joint DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

##########################
# building documentation #
##########################
build_doxygen_documentation()
