cmake_minimum_required(VERSION 2.8.3)
project(iiam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES iiam_library
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(iiam_library src/iiam.cpp
                         src/gridclass.cpp
                         src/shortgrid.cpp
                         src/vectorclass.cpp
                         src/calculation.cpp)
target_link_libraries(iiam_library ${catkin_LIBRARIES})

