######################
# set up the project #
######################
cmake_minimum_required(VERSION 2.8.3)

project(blmc_robots)

# required to use std::shared_ptr in code
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wl,--no-as-needed")

# ensuring path to libraries are set during install
set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

############################
# Search for dependencies. #
############################
include(cmake/pkg-config.cmake)
include(cmake/eigen.cmake)

set(CATKIN_PKGS ${CATKIN_PKGS}
  blmc_drivers
  real_time_tools
)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PKGS})
search_for_eigen()

######################################################
# define the include directory of all ${CATKIN_PKGS} #
######################################################
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
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

##########################################
# export the package as a catkin package #
##########################################
catkin_package(
  LIBRARIES ${EXPORTED_LIBRAIRIES}
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_PKGS}
)

##########################
# building documentation #
##########################
set(BUILD_DOCUMENTATION OFF CACHE BOOL
    "Set to ON if you want to build the documentation")
if(BUILD_DOCUMENTATION)
  message(STATUS "building doxygen documentation for ${PROJECT_NAME}")
  FIND_PACKAGE(Doxygen)
  if (NOT DOXYGEN_FOUND)
    message(FATAL_ERROR
        "Doxygen is needed to build the documentation. "
        "Please install it correctly")
  endif()
  configure_file(Doxyfile.in ${PROJECT_BINARY_DIR}/Doxyfile  @ONLY IMMEDIATE)
  # the doxygen target is generated
  add_custom_target (${PROJECT_NAME}_doc ALL
    COMMAND ${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/Doxyfile
    SOURCES ${PROJECT_BINARY_DIR}/Doxyfile)
endif()

###############################################
# Add custom (non compiling) targets          #
# so non cpp files show up in QT_CREATOR IDE. #
###############################################
set(USE_QTCREATOR_${PROJECT_NAME} OFF CACHE BOOL
    "Set to ON if you want QtCreator to display all files in the project tree")
if(USE_QTCREATOR_${PROJECT_NAME})
    file(GLOB_RECURSE EXTRA_FILES
        ../*.h ../*.hh ../*.hpp ../*.py
        ../*.cf  ../*.yaml ../*.txt ../*.xml
        ../start ../*.launch ../*.urdf ../CATKIN_IGNORE ../*.in
    )
    #list(FILTER EXTRA_FILES EXCLUDE REGEX "build")
    add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY
        ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})
endif()
