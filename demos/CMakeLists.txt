#####################
# create some demos #
#####################


include_directories(
  ${PROJECT_SOURCE_DIR}/demos
)

#############################################
# Declare a simple macro to build the demos #
#############################################

macro(create_demo demo_name)

### demo off the 8 blmc motor test bench ####
add_executable(demo_${demo_name}
  demo_${demo_name}.cpp
)

target_link_libraries(demo_${demo_name}
  ${demo_name}
  ${catkin_LIBRARIES}
)

target_compile_definitions(demo_${demo_name} PUBLIC
  YAML_PARAMS="${CMAKE_CURRENT_SOURCE_DIR}/config/${demo_name}.yaml")

endmacro(create_demo demo_name)

###############
# build demos #
###############

create_demo(test_bench_8_motors)
create_demo(finger)
create_demo(quadruped)
create_demo(teststand)
