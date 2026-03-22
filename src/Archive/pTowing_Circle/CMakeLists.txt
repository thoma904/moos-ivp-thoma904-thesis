#--------------------------------------------------------
# The CMakeLists.txt for:                       pTowing_Circle
# Author(s):                              Tom Monaghan
#--------------------------------------------------------

SET(SRC
  Towing_Circle.cpp
  Towing_Circle_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pTowing_Circle ${SRC})

TARGET_LINK_LIBRARIES(pTowing_Circle
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

