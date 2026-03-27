#--------------------------------------------------------
# The CMakeLists.txt for:                    app_aof_bench
# Author(s):                              Tom Monaghan
#--------------------------------------------------------

# Set System Specific Libraries
if (${WIN32})
  SET(SYSTEM_LIBS wsock32)
else (${WIN32})
  SET(SYSTEM_LIBS m)
endif (${WIN32})

SET(SRC
  main.cpp
  ${CMAKE_SOURCE_DIR}/src/lib_behaviors-test/AOF_TowObstacleAvoid.cpp
)

ADD_EXECUTABLE(aof_bench ${SRC})

TARGET_LINK_LIBRARIES(aof_bench
  mbutil
  geometry
  bhvutil
  behaviors
  helmivp
  ivpbuild
  logic
  ivpcore
  mbutil
  geometry
  bhvutil
  behaviors
  helmivp
  ivpbuild
  logic
  ivpcore
  ${SYSTEM_LIBS}
)
