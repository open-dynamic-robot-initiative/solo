#####################
# create some demos #
#####################

include_directories(
  ${PROJECT_SOURCE_DIR}/demos
  ${blmc_drivers_INCLUDE_DIRS}
)

#### demo with a real single motor ####
add_executable(demo_single_motor
  demo_single_motor.cpp
  single_motor/dgm_single_motor.cpp
)
target_link_libraries(demo_single_motor
  dynamic_graph_manager
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
#  rt
)
target_compile_definitions(demo_single_motor PUBLIC
  YAML_PARAMS="${CMAKE_CURRENT_SOURCE_DIR}/single_motor/dgm_single_motor.yaml")

#### demo with a simulated single motor ####
add_executable(demo_single_motor_simu
  demo_single_motor.cpp
  single_motor/dgm_single_motor.cpp
)
target_link_libraries(demo_single_motor_simu
  dynamic_graph_manager
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
#  rt
)
target_compile_definitions(demo_single_motor_simu PUBLIC
  YAML_PARAMS="${CMAKE_CURRENT_SOURCE_DIR}/single_motor/dgm_single_motor_simu.yaml")


# then we build the different entities and install them in devel/lib/plugin folder
set(LIST_MOTOR_ENTITY
  dgm_single_motor_controller
)

foreach(MOTOR_ENTITY ${LIST_MOTOR_ENTITY})
  add_library(${MOTOR_ENTITY} SHARED
    single_motor/${MOTOR_ENTITY}.cpp
  )
  target_link_libraries(${MOTOR_ENTITY}
    dynamic_graph_manager
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
  )
  set_target_properties(${MOTOR_ENTITY} PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY ${DYNAMIC_GRAPH_PLUGIN_DIR}
  )
  dynamic_graph_python_module("demos/single_motor/${MOTOR_ENTITY}"
    ${MOTOR_ENTITY}
    ${MOTOR_ENTITY}_wrap
  )
endforeach()
