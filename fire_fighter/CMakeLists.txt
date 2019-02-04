cmake_minimum_required(VERSION 2.8.3)
project(fire_fighter)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tough_controller_interface
  tough_common
  tough_moveit_planners
  geometry_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fire_fighter
  CATKIN_DEPENDS 	roscpp
                        rospy
                        std_msgs
                        tough_controller_interface
                        tough_common
                        tough_moveit_planners
                        geometry_msgs
#  DEPENDS system_lib
)


include_directories(${catkin_INCLUDE_DIRS} include)

add_library(fire_fighter
    src/manipulation_planner_node.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

######################
## Control examples ##
######################

## Test arm navigation
add_executable(manipulation_node src/manipulation_node.cpp)
add_dependencies(manipulation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(manipulation_node  ${catkin_LIBRARIES} ${PROJECT_NAME})

## Test arm navigation
add_executable(trial_manipulation_node src/trial_manipulation_node.cpp)
add_dependencies(trial_manipulation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trial_manipulation_node  ${catkin_LIBRARIES} ${PROJECT_NAME})

## CSV trial 
#add_executable(testCsv src/testCsv.cpp)
#add_dependencies(testCsv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(testCsv  ${catkin_LIBRARIES})
