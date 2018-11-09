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
endif()

set(EXPORTED_LIBRAIRIES_LOCAL ${EXPORTED_LIBRAIRIES_LOCAL} test_bench_8_motors)


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
endif()

set(EXPORTED_LIBRAIRIES_LOCAL ${EXPORTED_LIBRAIRIES_LOCAL} finger)

set(EXPORTED_LIBRAIRIES ${EXPORTED_LIBRAIRIES_LOCAL} PARENT_SCOPE)

