cmake_minimum_required(VERSION 2.8.3)
project(line_lbd)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV 3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES line_lbd_lib
  CATKIN_DEPENDS cmake_modules roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(${OpenCV_INCLUDE_DIRS})
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS
   "${CMAKE_CXX_FLAGS} ${SSE_FLAGS} -march=native  -std=c++0x"
) 

## Declare a C++ library
add_library(line_lbd_lib
  libs/binary_descriptor.cpp libs/binary_descriptor_matcher.cpp libs/LSDDetector.cpp  libs/lsd.cpp libs/bitarray.hpp  
  libs/bitops.hpp libs/draw.cpp libs/precomp.hpp libs/types.hpp  class/line_lbd_allclass.cpp
)
target_link_libraries(line_lbd_lib ${OpenCV_LIBRARIES} ${catkin_LIBRARIES}) 


add_executable(linelbd_detect_node src/detect_lines.cpp)
target_link_libraries(linelbd_detect_node  line_lbd_lib ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})


#add_executable(linelbd_descrip_node src/compute_descriptors.cpp)
#target_link_libraries(linelbd_descrip_node  line_lbd_lib ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

