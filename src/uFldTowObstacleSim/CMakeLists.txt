#--------------------------------------------------------
# The CMakeLists.txt for:                 uFldTowObstacleSim
# Author(s):                              Tom Monaghan
#--------------------------------------------------------

SET(SRC
  TowObstacleSim.cpp
  TowObstacleSim_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(uFldTowObstacleSim ${SRC})

TARGET_LINK_LIBRARIES(uFldTowObstacleSim
   ${MOOS_LIBRARIES}
   ${MOOSGeodesy_LIBRARIES}
   apputil
   obstacles
   geodaid
   contacts
   geometry
   mbutil
   m
   pthread)
