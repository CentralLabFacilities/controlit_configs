cmake_minimum_required(VERSION 2.8.3)
project(controlit_gazebo_plugins)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    shared_memory_interface
    controlit_robot_interface_library
)

find_package(gazebo REQUIRED)

find_package(Boost REQUIRED COMPONENTS 
    thread 
    filesystem 
    program_options
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