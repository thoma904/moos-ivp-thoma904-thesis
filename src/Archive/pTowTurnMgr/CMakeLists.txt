#--------------------------------------------------------
# The CMakeLists.txt for:                       pTowTurnMgr
# Author(s):                              Tom Monaghan
#--------------------------------------------------------

SET(SRC
  TowTurnMgr.cpp
  TowTurnMgr_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pTowTurnMgr ${SRC})

TARGET_LINK_LIBRARIES(pTowTurnMgr
   ${MOOS_LIBRARIES}
   geometry
   apputil
   mbutil
   m
   pthread)

