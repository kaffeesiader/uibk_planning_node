cmake_minimum_required(VERSION 2.8.3)
project(uibk_planning_node)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

## find_package(catkin REQUIRED COMPONENTS moveit_core moveit_ros_planning moveit_ros_planning_interface pluginlib actionlib cmake_modules)
find_package(catkin REQUIRED roscpp moveit_msgs sensor_msgs eigen moveit_core moveit_ros_planning_interface geometry_msgs definitions actionlib)
## find_package(Boost REQUIRED system filesystem date_time thread)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

set(CMAKE_CXX_FLAGS "-std=c++0x")

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES uibk_planning_node
# CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
#include_directories(include
#	${PACMAN}/pacman/include
#	${catkin_INCLUDE_DIRS}
#	)
include_directories(
   include
   /usr/include/eigen3
   ${catkin_INCLUDE_DIRS}
   ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(planning_helper src/PlanningHelper.cpp)
target_link_libraries(planning_helper ${catkin_LIBRARIES})

add_library(pickup_planner src/pickupplanner.cpp)
add_dependencies(pickup_planner definitions_generate_messages_cpp)
target_link_libraries(pickup_planner ${catkin_LIBRARIES})

add_library(cart_planner src/CartPlanner.cpp)
add_dependencies(cart_planner definitions_generate_messages_cpp)
target_link_libraries(cart_planner ${catkin_LIBRARIES})

add_library(state_planner src/StatePlanner.cpp)
add_dependencies(state_planner definitions_generate_messages_cpp)
target_link_libraries(state_planner ${catkin_LIBRARIES})

add_library(kinematics_helper src/KinematicsHelper.cpp)
target_link_libraries(kinematics_helper ${catkin_LIBRARIES})

add_library(trajectory_planner src/TrajectoryPlannerImpr.cpp)
target_link_libraries(trajectory_planner kinematics_helper ${catkin_LIBRARIES})

add_library(visualization_tools src/VisualizationTools.cpp)
target_link_libraries(visualization_tools ${catkin_LIBRARIES})

## Declare a cpp executable
# add_executable(uibk_planning_node_node src/uibk_planning_node_node.cpp)

#add_executable(plan_execution_node src/PlanExecutionNode.cpp)
#add_dependencies(plan_execution_node definitions_generate_messages_cpp planning_helper)
#target_link_libraries(plan_execution_node planning_helper ${catkin_LIBRARIES})

#add_executable(test_plan_execution_node tests/test_plan_execution_node.cpp)
#add_dependencies(test_plan_execution_node definitions_generate_messages_cpp)
#target_link_libraries(test_plan_execution_node ${catkin_LIBRARIES})

add_executable(move_goal src/move_goal.cpp)
target_link_libraries(move_goal ${catkin_LIBRARIES})

add_executable(trajectory_planner_node src/trajectory_planner_node.cpp)
add_dependencies(trajectory_planner_node
    pickup_planner
    state_planner
    cart_planner
    definitions_generate_messages_cpp
)
target_link_libraries(trajectory_planner_node
    pickup_planner
    cart_planner
    state_planner
    ${catkin_LIBRARIES}
)

add_executable(ping_gripper_only tests/ping_gripper_only.cpp)
add_dependencies(ping_gripper_only definitions_generate_messages_cpp)
target_link_libraries(ping_gripper_only ${catkin_LIBRARIES})

add_executable(test_pickup_planner tests/test_pickup_planner.cpp)
add_dependencies(test_pickup_planner definitions_generate_messages_cpp)
target_link_libraries(test_pickup_planner ${catkin_LIBRARIES})

add_executable(kinematics_helper_tests tests/kinematics_helper_tests.cpp)
add_dependencies(kinematics_helper_tests kinematics_helper)
target_link_libraries(kinematics_helper_tests kinematics_helper ${catkin_LIBRARIES})

add_executable(test_trajectory_planner tests/test_trajectory_planner.cpp)
add_dependencies(test_trajectory_planner kinematics_helper trajectory_planner)
target_link_libraries(test_trajectory_planner kinematics_helper trajectory_planner ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(uibk_planning_node_node uibk_planning_node_generate_messages_cpp)

# executables
#add_dependencies(test_plan_execution_node definitions_generate_messages_cpp)
#add_dependencies(plan_execution_node definitions_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(uibk_planning_node_node
#   ${catkin_LIBRARIES}
# )

#


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS uibk_planning_node uibk_planning_node_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_uibk_planning_node.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
