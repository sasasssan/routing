cmake_minimum_required(VERSION 2.8.3)
project(routing_driver)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  costmap_2d
  roscpp
  sensor_msgs
  visualization_msgs
  tf
)

find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES routing_driver
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs visualization_msgs tf
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
)

add_executable(grid_routing src/grid_routing.cpp)
target_link_libraries(grid_routing
  ${catkin_LIBRARIES}
  yaml-cpp
)

add_executable(check_mapgrid_info src/check_mapgrid_info.cpp)
target_link_libraries(check_mapgrid_info
  ${catkin_LIBRARIES}
)