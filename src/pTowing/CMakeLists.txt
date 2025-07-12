#--------------------------------------------------------
# The CMakeLists.txt for:                       pTowing
# Author(s):                              Tom Monaghan
#--------------------------------------------------------

SET(SRC
  Towing.cpp
  Towing_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pTowing ${SRC})

TARGET_LINK_LIBRARIES(pTowing
   ${MOOS_LIBRARIES}
   geometry
   apputil
   mbutil
   m
   pthread)

