#
# Python bindings.
#

# solo12 python module
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


if(${dynamic_graph_manager_FOUND})
    # solo12 hardware process python module
    add_library(solo12_hwp MODULE
        ../src/dynamic_graph_manager/dgm_solo12.cpp
        solo12_hwp.cpp
    )
    target_link_libraries(solo12_hwp PRIVATE pybind11::module)
    target_link_libraries(solo12_hwp PRIVATE ${PYTHON_LIBRARIES})
    target_link_libraries(solo12_hwp PRIVATE ${PROJECT_NAME})
    target_link_libraries(solo12_hwp PRIVATE solo12)
    set_target_properties(solo12_hwp PROPERTIES PREFIX ""
                                            SUFFIX "${PYTHON_MODULE_EXTENSION}")
    target_include_directories(
    solo12_hwp
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include> SYSTEM
    PUBLIC ${PYTHON_INCLUDE_DIRS})
    _ament_cmake_python_get_python_install_dir()
    install(TARGETS solo12_hwp DESTINATION ${PYTHON_INSTALL_DIR})
endif()
