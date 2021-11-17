cmake_minimum_required(VERSION 2.8.3)
project(utils_sim)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  sensor_msgs
  gazebo_msgs
  tf
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS include
#    LIBRARIES msg2tf
    CATKIN_DEPENDS nav_msgs sensor_msgs tf gazebo_msgs
#    DEPENDS 
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(gazebo2odom_node src/gazebo2odom.cpp)

## Rename C++ executable without prefix
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(gazebo2odom_node PROPERTIES OUTPUT_NAME gazebo2odom PREFIX "")

## Add cmake target dependencies of the executable
add_dependencies(gazebo2odom_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(gazebo2odom_node
    ${catkin_LIBRARIES}
)


## Declare a C++ executable
add_executable(lin_vel_control_node src/lin_vel_control.cpp)

## Rename C++ executable without prefix
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(lin_vel_control_node PROPERTIES OUTPUT_NAME lin_vel_control PREFIX "")

## Add cmake target dependencies of the executable
add_dependencies(lin_vel_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(lin_vel_control_node
    ${catkin_LIBRARIES}
)

## Declare a C++ executable
add_executable(lidar_nearest src/lidar_nearest.cpp)

## Rename C++ executable without prefix
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(lidar_nearest PROPERTIES OUTPUT_NAME lidar_nearest PREFIX "")

## Add cmake target dependencies of the executable
add_dependencies(lidar_nearest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(lidar_nearest
    ${catkin_LIBRARIES}
)
#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS gazebo2odom_node lin_vel_control_node lidar_nearest
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
#install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
#install(DIRECTORY cfg DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg)
