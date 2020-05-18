###################################################
## Set a general config folder path for all tests #
###################################################
set(TEST_CONFIG_PATH ${PROJECT_SOURCE_DIR}/tests/config/)

###################
## add unit tests #
###################

macro(create_blmc_robots_unittest test_name)

# create the executable
catkin_add_gtest(${test_name}_ut
  main.cpp
  test_${test_name}.cpp
)
# link the dependecies to it
target_link_libraries(${test_name}_ut
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)
# add some preprocessor variable
target_compile_definitions(${test_name}_ut PUBLIC
  TEST_CONFIG_PATH="${PROJECT_SOURCE_DIR}/tests/config/"
)

endmacro(create_blmc_robots_unittest test_name)

create_blmc_robots_unittest(polynomes)
create_blmc_robots_unittest(process_action)
