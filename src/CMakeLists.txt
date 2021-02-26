set(all_src_targets)

#
# Declare the project library.
#
add_library(${PROJECT_NAME} INTERFACE)
# Add the include dependencies.
target_include_directories(
  ${PROJECT_NAME} INTERFACE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>)
# Link the dependencies.
target_link_libraries(${PROJECT_NAME} INTERFACE ati_ft_sensor::ati_ft_sensor)
target_link_libraries(${PROJECT_NAME} INTERFACE blmc_drivers::blmc_drivers)
target_link_libraries(${PROJECT_NAME}
                      INTERFACE odri_control_interface::odri_control_interface)
target_link_libraries(${PROJECT_NAME}
                      INTERFACE master_board_sdk::master_board_sdk)
target_link_libraries(${PROJECT_NAME}
                      INTERFACE real_time_tools::real_time_tools)
target_link_libraries(${PROJECT_NAME} INTERFACE yaml_utils::yaml_utils)
target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)
# Export the target.
list(APPEND all_src_targets ${PROJECT_NAME})

#
# `create_robots_library` builds a robot driver library.
#
macro(create_robots_library lib_name)
  add_library(${lib_name} SHARED ${lib_name}.cpp)
  # Add the include dependencies.
  target_include_directories(
    ${lib_name} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                       $<INSTALL_INTERFACE:include>)
  # Link the dependencies.
  target_link_libraries(${lib_name} ${PROJECT_NAME})
  # Export the target.
  list(APPEND all_src_targets ${lib_name})
endmacro()
create_robots_library(test_bench_8_motors)
create_robots_library(solo8)
create_robots_library(solo8ti)
create_robots_library(solo12)
create_robots_library(teststand)
create_robots_library(stuggihop)
create_robots_library(single_motor)
create_robots_library(single_leg)

#
# Build executables like the calibration programs.
#
macro(build_programs program_name robot_name)
  add_executable(${program_name} programs/${program_name}.cpp)
  # Add the include dependencies.
  target_include_directories(
    ${program_name} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:include>)
  # Link the dependencies.
  target_link_libraries(${program_name} ${robot_name})
  # Export the target.
  list(APPEND all_src_targets ${program_name})
endmacro()
build_programs(solo8_hardware_calibration solo8)
build_programs(solo8ti_hardware_calibration solo8ti)
build_programs(solo12_hardware_calibration solo12)
build_programs(teststand_hardware_calibration teststand)

#
# Install and Export the libraries.
#
install(
  TARGETS ${all_src_targets}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES
  DESTINATION include)
