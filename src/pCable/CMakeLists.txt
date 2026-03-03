#--------------------------------------------------------
# The CMakeLists.txt for:                       pCable
# Author(s):                              Tom Monaghan
#--------------------------------------------------------

SET(SRC
  Cable.cpp
  Cable_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pCable ${SRC})

TARGET_LINK_LIBRARIES(pCable
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

