###############################################
# Declare a simple macro to build the library #
###############################################

macro(create_robots_library lib_name)

add_library(${lib_name} SHARED ${lib_name}.cpp)

target_link_libraries(${lib_name} ${catkin_LIBRARIES})

if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  target_link_libraries(${lib_name} rt pthread)
endif()

set(LIBRARIES_TO_BE_EXPORTED ${LIBRARIES_TO_BE_EXPORTED} ${lib_name})

endmacro()

####################
# build the Robots #
####################

create_robots_library(test_bench_8_motors)
create_robots_library(finger)
create_robots_library(quadruped)

######################################################
# Export the libraries in the devel folder of catkin #
######################################################

set(EXPORTED_LIBRAIRIES ${EXPORTED_LIBRAIRIES} ${LIBRARIES_TO_BE_EXPORTED}
    PARENT_SCOPE)
