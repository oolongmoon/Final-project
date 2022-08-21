################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_navigation)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
        roscpp
		rospy
        std_msgs
		message_generation
		geometry_msgs
        )

add_service_files(
  FILES
  info.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package()
################################################################################
# Build
################################################################################
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
)

add_executable(avoid_0 src/avoid_0.cpp src/avoid.cpp)
add_dependencies(avoid_0 ${avoid_EXPORTED_TARGET} ${catkin_EXPORTED_TARGETS} ${turtlebot3_navigation_EXPORTED_TARGET})
target_link_libraries(avoid_0 ${catkin_LIBRARIES})

add_executable(global src/global.cpp)
add_dependencies(global ${global_EXPORTED_TARGET} ${catkin_EXPORTED_TARGETS} ${turtlebot3_navigation_EXPORTED_TARGET})
target_link_libraries(global ${catkin_LIBRARIES})
## Declare a C++ library

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(simple_local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(simple_local_planner_node src/simple_local_planner_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(simple_local_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#target_link_libraries(local_planner
#        ${catkin_LIBRARIES}
#        )

################################################################################
# Install
################################################################################
install(DIRECTORY launch maps param rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
################################################################################
# Test
################################################################################
