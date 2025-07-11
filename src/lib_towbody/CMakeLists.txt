#--------------------------------------------------------
# The CMakeLists.txt for:                     lib_towbody
# Author(s):                                Mike Benjamin
#--------------------------------------------------------

SET(SRC
  XYBezier.cpp
  TowBodyModel.cpp
)

SET(HEADERS
  XYBezier.h
  TowBodyModel.h
)

# Build Library
ADD_LIBRARY(towbody ${SRC})

TARGET_LINK_LIBRARIES(towbody
   mbutil
)

