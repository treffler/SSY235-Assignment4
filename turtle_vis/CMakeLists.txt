cmake_minimum_required(VERSION 2.8.3)
project(turtle_vis)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_compile_options(-Wno-dev -Wall -Wextra -Wpedantic)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    tf
    tf_conversions
    visualization_msgs
    message_generation
    )

## System dependencies are found with CMake's conventions
##########################################################################
# Eigen
##########################################################################
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

##########################################################################
# Messages, Services and Actions
##########################################################################
##>>>>TODO: Generate messages in the 'msg' folder
#add_message_files(
#    DIRECTORY msg
#    FILES
#     #>>>>TODO: ADD THE MSG FILES
#    std_msgs.msg
#    )
##>>>>TODO: Generate services in the 'srv' folder
#add_service_files(
#    DIRECTORY srv
#    FILES
#    #>>>>TODO: ADD THE SERVICE FILES
#    std_srvs.srv
#    )


##>>>>TODO:Generate added messages and services with any dependencies listed here
#generate_messages(
#    DEPENDENCIES
#    std_msgs
#    )

#generate_messages(
#    DEPENDENCIES
#    std_srvs
#    )

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
    LIBRARIES turtle_ros_tutorial
    CATKIN_DEPENDS
    roscpp
    std_msgs
    tf
    tf_conversions
    visualization_msgs
    message_runtime
    )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    )
###########################################################################
# Sections Binaries (Libraries)
###########################################################################
# >>>>TODO: Define the library target
#add_library(turtle_ros_tutorial src/TurtleClass.cpp)
#target_link_libraries(turtle_ros_tutorial ${catkin_LIBRARIES})
#add_dependencies(turtle_ros_tutorial 
#    ${${PROJECT_NAME}_EXPORTED_TARGETS}
#    ${catkin_EXPORTED_TARGETS})


###########################################################################
# Sections Library Enable for Releases
###########################################################################
#link_directories(${PROJECT_SOURCE_DIR}/lib)

###########################################################################
# Sections Binaries (C++ Exec)
###########################################################################
##>>>>TODO: enable executables
#add_executable(turtle_vis_node src/applications/turtle_vis_node.cpp)
#add_dependencies(turtle_vis_node 
#    ${${PROJECT_NAME}_EXPORTED_TARGETS}
#    ${catkin_EXPORTED_TARGETS})
#target_link_libraries(turtle_vis_node
#    ${catkin_LIBRARIES}
#    turtle_ros_tutorial)

#add_executable(turtle_control_node src/applications/turtle_control_node.cpp)
#add_dependencies(turtle_control_node 
#    ${${PROJECT_NAME}_EXPORTED_TARGETS}
#    ${catkin_EXPORTED_TARGETS})
#target_link_libraries(turtle_control_node
#    ${catkin_LIBRARIES}
#    turtle_ros_tutorial)

#add_executable(turtle_set_position_node src/applications/turtle_set_position.cpp)
#add_dependencies(turtle_set_position_node
#    ${${PROJECT_NAME}_EXPORTED_TARGETS}
#    ${catkin_EXPORTED_TARGETS})
#target_link_libraries(turtle_set_position_node
#    ${catkin_LIBRARIES})



#############
## Install ##
#############

## C++ executables
##>>>>TODO: Uncomment these lines
#install(TARGETS
#    turtle_vis_node
#    turtle_control_node
#    turtle_set_position_node
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

##>>>>TODO: Uncomment these lines
## Libraries
#install(TARGETS turtle_ros_tutorial
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

## Header files
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".git" EXCLUDE)

## Other files and dirs
install(DIRECTORY meshes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY configs/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/configs
  PATTERN ".svn" EXCLUDE)



















