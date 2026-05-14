#--------------------------------------------------------
# The CMakeLists.txt for:                       pCableLengthTracker
# Author(s):                              Tom Monaghan
#--------------------------------------------------------

SET(SRC
  CableLengthTracker.cpp
  CableLengthTracker_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pCableLengthTracker ${SRC})

TARGET_LINK_LIBRARIES(pCableLengthTracker
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

