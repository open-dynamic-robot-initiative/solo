#
# Declare a simple macro to build the demos.
#

set(all_demo_targets)

macro(create_demo demo_name)
  # Cannot use ARGN directly with list() command, so copy it to a variable
  # first.
  set(extra_macro_args ${ARGN})

  # Did we get any optional args?
  list(LENGTH extra_macro_args num_extra_args)
  if(${num_extra_args} GREATER 0)
    list(GET extra_macro_args 0 robot_name)
  endif()
  if(${num_extra_args} GREATER 1)
    list(GET extra_macro_args 1 yaml_path)
  endif()

  # Check if the given config exists. Otherwise do not compile.
  add_executable(demo_${demo_name} demo_${demo_name}.cpp common_demo_header.hpp)
  # Add the include dependencies.
  target_include_directories(
    demo_${demo_name} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                             $<INSTALL_INTERFACE:include>)
  # Link the dependecies to it.
  target_link_libraries(demo_${demo_name} ${robot_name} ${PROJECT_NAME})

  # Export targets
  list(APPEND all_demo_targets demo_${demo_name})

  if(yaml_path)
    if(EXISTS ${yaml_path})
      target_compile_definitions(
        demo_${demo_name} PUBLIC ROBOT_PROPERTIES_YAML_PATH="${yaml_path}")
    else()
      message(FATAL_ERROR "The yaml path ${yaml_path} is invalid!")
    endif()
  endif()
endmacro(create_demo demo_name)

#
# Build the demos.
#
create_demo(test_bench_8_motors test_bench_8_motors)
create_demo(solo8 solo8)
create_demo(solo8ti solo8ti)
create_demo(
  solo12
  solo12
  ${PythonModules_robot_properties_solo_PATH}/robot_properties_solo/robot_properties_solo/dynamic_graph_manager/dgm_parameters_solo12.yaml
)
create_demo(teststand teststand)
create_demo(arduino_slider)
create_demo(
  solo8_calibration
  solo8
  ${PythonModules_robot_properties_solo_PATH}/robot_properties_solo/robot_properties_solo/dynamic_graph_manager/dgm_parameters_solo8.yaml
)
create_demo(
  teststand_calibration
  teststand
  ${PythonModules_robot_properties_teststand_PATH}/robot_properties_teststand/robot_properties_teststand/dynamic_graph_manager/dgm_parameters_teststand.yaml
)

#
# Install and Export the libraries.
#
install(
  TARGETS ${all_demo_targets}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES
  DESTINATION include)
