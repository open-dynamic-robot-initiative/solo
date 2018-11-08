##########################################
# create the test_bench_8_motors library #
##########################################

set(SOURCES
  test_bench_8_motors/test_bench_8_motors.cpp
)
add_library(test_bench_8_motors SHARED ${SOURCES})
target_link_libraries(test_bench_8_motors ${catkin_LIBRARIES})

if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
target_link_libraries(test_bench_8_motors
  rt
  pthread
)
endif(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

set(EXPORTED_LIBRAIRIES ${EXPORTED_LIBRAIRIES} test_bench_8_motors
    PARENT_SCOPE)


#############################
# create the finger library #
#############################

set(SOURCES
  finger/finger.cpp
)
add_library(finger SHARED ${SOURCES})
target_link_libraries(finger ${catkin_LIBRARIES})

if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
target_link_libraries(finger
  rt
  pthread
)
endif(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


set(EXPORTED_LIBRAIRIES ${EXPORTED_LIBRAIRIES} finger
    PARENT_SCOPE)
