###################
# python bindings #
###################

##
set(py_blmc_single_leg_SRC_FILES
  ../src/single_leg.cpp
  py_single_leg.cpp
)
pybind11_add_module(blmc_single_leg ${py_blmc_single_leg_SRC_FILES})
target_link_libraries(blmc_single_leg PRIVATE single_leg)
set_target_properties(blmc_single_leg PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
install(TARGETS blmc_single_leg DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

##
set(py_solo12_SRC_FILES
  py_solo12.cpp
)
pybind11_add_module(py_solo12 ${py_solo12_SRC_FILES})
target_link_libraries(py_solo12 PRIVATE solo12 ${catkin_LIBRARIES} ${PROJECT_NAME} ${YAML_CPP_LIBRARIES})
set_target_properties(py_solo12 PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
install(TARGETS py_solo12 DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})
