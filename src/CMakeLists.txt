##################################
# create the twin_motors library #
##################################

set(SOURCES
)
add_library(twin_motors SHARED ${SOURCES})
target_link_libraries(twin_motors ${catkin_LIBRARIES})
if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
target_link_libraries(twin_motors
  rt
  pthread
)
endif(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

set(EXPORTED_LIBRAIRIES ${EXPORTED_LIBRAIRIES} twin_motors
    PARENT_SCOPE)
