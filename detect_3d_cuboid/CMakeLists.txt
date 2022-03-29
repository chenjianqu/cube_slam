cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
project(detect_3d_cuboid)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  roslib
  tictoc_profiler
)


find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cmake_modules roscpp roslib tictoc_profiler
#  DEPENDS system_lib
)


include_directories(include)
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}  
)

SET(CMAKE_BUILD_TYPE Release)

add_library(${PROJECT_NAME}  SHARED
src/box_proposal_detail.cpp src/object_3d_util.cpp src/matrix_utils.cpp  
)
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES}
)

add_executable(detect_3d_cuboid_node src/main.cpp)
target_link_libraries(detect_3d_cuboid_node  ${PROJECT_NAME}
   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES}
)

