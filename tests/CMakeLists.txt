###################################################
## Set a general config folder path for all tests #
###################################################
set(TEST_CONFIG_PATH ${PROJECT_SOURCE_DIR}/tests/config/)

###################
## add unit tests #
###################

# create the executable
catkin_add_gtest(test_dynamic_graph_manager
  main.cpp
  test_dynamic_graph_manager.cpp
)
# link the dependecies to it
target_link_libraries(test_dynamic_graph_manager
    dynamic_graph_manager
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)
# add some preprocessor variable
target_compile_definitions(test_dynamic_graph_manager PUBLIC
  TEST_CONFIG_FOLDER="${TEST_CONFIG_PATH}"
)

###

# create the executable
catkin_add_gtest(test_device
  main.cpp
  test_device.cpp
)
# link the dependecies to it
target_link_libraries(test_device
    dynamic_graph_manager
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
#    rt
)
# add some preprocessor variable
target_compile_definitions(test_device PUBLIC
  TEST_CONFIG_FOLDER="${TEST_CONFIG_PATH}"
)
