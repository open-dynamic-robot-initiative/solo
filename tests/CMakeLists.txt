#
# Add unit tests.
#

# Use gtest
find_package(ament_cmake_gtest)

# Create a usefull macro to build different unit tests suits.
macro(create_blmc_robots_unittest test_name)

  set(test_target_name test_${test_name})

  # create the executable
  ament_add_gtest(${test_target_name} main.cpp ${test_target_name}.cpp)
  # Add the include dependencies.
  target_include_directories(
    ${test_target_name}
    PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  # link the dependecies to it
  target_link_libraries(${test_target_name} ${PROJECT_NAME})
  # add some preprocessor variable
  target_compile_definitions(
    ${test_target_name}
    PUBLIC TEST_CONFIG_PATH="${PROJECT_SOURCE_DIR}/tests/config/")

endmacro(create_blmc_robots_unittest test_name)

create_blmc_robots_unittest(polynomes)
create_blmc_robots_unittest(process_action)
create_blmc_robots_unittest(n_joint_blmc_robot_driver)
create_blmc_robots_unittest(clamp)
