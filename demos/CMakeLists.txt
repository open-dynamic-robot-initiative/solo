#####################
# create some demos #
#####################

include_directories(
  ${PROJECT_SOURCE_DIR}/demos
)

#### demo off the 8 blmc motor test bench ####
add_executable(demo_test_bench_8_motors
  test_bench_8_motors/demo_test_bench_8_motors.cpp
)

target_link_libraries(demo_test_bench_8_motors
  test_bench_8_motors
  ${catkin_LIBRARIES}
)
