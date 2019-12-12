#####################
# create some demos #
#####################


include_directories(
  ${PROJECT_SOURCE_DIR}/demos
)

#############################################
# Declare a simple macro to build the demos #
#############################################

macro(create_demo demo_name robot_name)
  add_executable(demo_${demo_name}
    demo_${demo_name}.cpp
    common_demo_header.hpp
  )

  target_link_libraries(demo_${demo_name}
    ${robot_name}
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
  )

  # first look if a robot_properties_${robot_name} package contain a config file.
  set(config_file
    "${robot_properties_${robot_name}_SOURCE_DIR}/config/dgm_parameters.yaml"
  )
  target_compile_definitions(demo_${demo_name} PUBLIC
    YAML_PARAMS="${config_file}")

endmacro(create_demo demo_name)

###############
# build demos #
###############

create_demo(test_bench_8_motors test_bench_8_motors)
create_demo(solo8 solo8)
create_demo(solo12 solo12)
create_demo(teststand teststand)
create_demo(stuggihop stuggihop)

create_demo(solo8_calibration solo8)
create_demo(solo12_calibration solo12)
create_demo(teststand_calibration teststand)

create_demo(finger_position_control "")
create_demo(finger_torque_control "")
