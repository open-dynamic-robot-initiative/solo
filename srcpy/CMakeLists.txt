#
# Python bindings.
#

# blmc_single_leg python module
add_library(blmc_single_leg MODULE ../src/single_leg.cpp py_single_leg.cpp)
target_link_libraries(blmc_single_leg PRIVATE pybind11::module)
target_link_libraries(blmc_single_leg PRIVATE ${PYTHON_LIBRARIES})
target_link_libraries(blmc_single_leg PRIVATE ${PROJECT_NAME})
set_target_properties(blmc_single_leg
                      PROPERTIES PREFIX "" SUFFIX "${PYTHON_MODULE_EXTENSION}")
target_include_directories(
  blmc_single_leg
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include> SYSTEM
  PUBLIC ${PYTHON_INCLUDE_DIRS})
_ament_cmake_python_get_python_install_dir()
install(TARGETS blmc_single_leg DESTINATION ${PYTHON_INSTALL_DIR})

# blmc_single_leg python module
add_library(py_solo12 MODULE py_solo12.cpp)
target_link_libraries(py_solo12 PRIVATE pybind11::module)
target_link_libraries(py_solo12 PRIVATE ${PYTHON_LIBRARIES})
target_link_libraries(py_solo12 PRIVATE ${PROJECT_NAME})
target_link_libraries(py_solo12 PRIVATE solo12)
set_target_properties(py_solo12 PROPERTIES PREFIX ""
                                           SUFFIX "${PYTHON_MODULE_EXTENSION}")
target_include_directories(
  py_solo12
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include> SYSTEM
  PUBLIC ${PYTHON_INCLUDE_DIRS})
_ament_cmake_python_get_python_install_dir()
install(TARGETS py_solo12 DESTINATION ${PYTHON_INSTALL_DIR})
