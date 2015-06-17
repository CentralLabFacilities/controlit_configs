cmake_minimum_required(VERSION 2.8.3)
project(controlit_simulation_gazebo)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    shared_memory_interface
    controlit_cmake
    controlit_robot_interface_library
    message_generation
    geometry_msgs
)

find_package(gazebo REQUIRED)

find_package(Boost REQUIRED COMPONENTS 
    thread 
    filesystem 
    program_options
)

# The following is necessary to set the build type.
list(APPEND CMAKE_MODULE_PATH ${controlit_cmake_DIR}/../../../../src/controlit/controlit_cmake/cmake)
include(controlitbuild)
controlit_build_init()

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
add_message_files(
    FILES
    Wrenches.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

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
#  INCLUDE_DIRS include
#  LIBRARIES controlit
  CATKIN_DEPENDS controlit_robot_interface_library
#  DEPENDS system_lib
)

include_directories(
    ${GAZEBO_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
endif()

#uncomment if you have defined messages
# rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# include (FindPkgConfig)
# if (PKG_CONFIG_FOUND)
#   pkg_check_modules(GAZEBO gazebo REQUIRED)
# else()
#   message(FATAL_ERROR "pkg-config is required; please install it")
# endif()

# message(STATUS "GAZEBO_INCLUDE_DIRS = " ${GAZEBO_INCLUDE_DIRS})
# message(STATUS "GAZEBO_LIBRARY_DIRS = " ${GAZEBO_LIBRARY_DIRS})

# include_directories(${GAZEBO_INCLUDE_DIRS})
# link_directories(${GAZEBO_LIBRARY_DIRS})


add_library(ControlItROSTopicPlugin 
    gazebo_plugins/src/ControlItROSTopicPlugin.cpp
)

target_link_libraries(ControlItROSTopicPlugin 
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
)

add_library(SMControlPlugin SHARED 
    gazebo_plugins/src/SMControlPlugin.cpp
)

target_link_libraries(SMControlPlugin 
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    ${Boost_LIBRARIES}
)

add_library(stick_bot_plugin 
    gazebo_plugins/src/stick_bot_plugin.cpp
)

target_link_libraries(stick_bot_plugin
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
)

add_library(PinPlugin 
    gazebo_plugins/src/PinPlugin.cpp
)

target_link_libraries(PinPlugin
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
)

add_library(WrenchMeasurementPlugin SHARED
    gazebo_plugins/src/WrenchMeasurementPlugin.cpp
)

add_dependencies(WrenchMeasurementPlugin
    controlit_simulation_gazebo_generate_messages_cpp
)

target_link_libraries(WrenchMeasurementPlugin
    ${GAZEBO_LIBRARIES}
    ${Boost_LIBRARIES}
)
