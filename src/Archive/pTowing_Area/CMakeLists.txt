#--------------------------------------------------------
# The CMakeLists.txt for:                       pTowing_Area
# Author(s):                              Tom Monaghan
#--------------------------------------------------------

SET(SRC
  Towing_Area.cpp
  Towing_Area_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pTowing_Area ${SRC})

TARGET_LINK_LIBRARIES(pTowing_Area
   ${MOOS_LIBRARIES}
   geometry
   apputil
   mbutil
   m
   pthread)

