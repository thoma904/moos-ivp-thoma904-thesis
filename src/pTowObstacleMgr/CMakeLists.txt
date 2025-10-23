#--------------------------------------------------------
# The CMakeLists.txt for:                       pTowObstacleMgr
# Author(s):                              Tom Monaghan
#--------------------------------------------------------

SET(SRC
  TowObstacleMgr.cpp
  TowObstacleMgr_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pTowObstacleMgr ${SRC})

TARGET_LINK_LIBRARIES(pTowObstacleMgr
   ${MOOS_LIBRARIES}
   apputil
   obstacles
   geometry
   mbutil
   m
   pthread)

