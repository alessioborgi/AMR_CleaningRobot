cmake_minimum_required(VERSION 2.8.3)
project(robot_settings)
set(CMAKE_CXX_STANDARD 11)

## Find packages.
find_package(catkin REQUIRED COMPONENTS
rospy
roscpp
std_msgs
webots_ros
sensor_msgs
geometry_msgs
)


# Catkin.
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_settings
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(
include
${catkin_INCLUDE_DIRS}
)

# Making executable of sensor enable.cpp namely "enable" which will be called in launch file.
add_executable(enable src/sensor_enable.cpp)
target_link_libraries(enable ${catkin_LIBRARIES})
add_dependencies (enable ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Making executable of static_camera.cpp namely "dynamic_link_camera" which will be called in launch file.
add_executable(dynamic_link_camera src/static_camera.cpp)
target_link_libraries(dynamic_link_camera ${catkin_LIBRARIES})
add_dependencies (dynamic_link_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Making executable of localization.cpp namely "localize" which will be caled in the launch file.
add_executable(localize src/localization.cpp)
target_link_libraries(localize ${catkin_LIBRARIES})
add_dependencies (localize ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})