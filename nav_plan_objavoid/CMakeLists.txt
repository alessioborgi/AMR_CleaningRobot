cmake_minimum_required(VERSION 3.0.2)
project(nav_plan_objavoid)

# Find Catkin macros and libraries.
find_package(catkin REQUIRED COMPONENTS
roscpp
std_msgs
webots_ros
sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES navigation
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(
include
${catkin_INCLUDE_DIRS}
)

add_executable(remap src/remapping.cpp)
target_link_libraries(remap ${catkin_LIBRARIES})
add_dependencies (remap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})