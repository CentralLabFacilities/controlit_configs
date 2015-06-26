cmake_minimum_required(VERSION 2.8.3)
project(dreamer_controlit)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    cmake_modules
    # interactive_markers
    controlit_dependency_addons
)

find_package(gazebo REQUIRED)
find_package(Eigen REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${controlit_cmake_DIR}/../../../../src/controlit/controlit_cmake/cmake)

include(controlitbuild)
controlit_build_init()

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
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#include (FindPkgConfig)
#if (PKG_CONFIG_FOUND)
#  pkg_check_modules(GAZEBO gazebo REQUIRED)
#else()
#  message(FATAL_ERROR "pkg-config is required; please install it")
#endif()

#message(STATUS ${GAZEBO_INCLUDE_DIRS})
#message(STATUS ${GAZEBO_LIBRARY_DIRS})

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(TestQuaternionTrajectory
    src/TestQuaternionTrajectory.cpp
)

target_link_libraries(TestQuaternionTrajectory
    ${catkin_LIBRARIES}
    #${GAZEBO_LIBRARIES}
)

add_library(DreamerTorsoTransmissionPlugin
    gazebo_plugins/src/DreamerTorsoTransmissionPlugin.cpp
)

target_link_libraries(DreamerTorsoTransmissionPlugin
    ${GAZEBO_LIBRARIES}
)

add_library(DreamerEyePlugin
    gazebo_plugins/src/DreamerEyePlugin.cpp
)

target_link_libraries(DreamerEyePlugin
    ${GAZEBO_LIBRARIES}
)

add_library(DreamerLeftGripperPlugin
    gazebo_plugins/src/DreamerLeftGripperPlugin.cpp
)

target_link_libraries(DreamerLeftGripperPlugin
    ${GAZEBO_LIBRARIES}
)

add_library(DreamerNeckPlugin
    gazebo_plugins/src/DreamerNeckPlugin.cpp
)

target_link_libraries(DreamerNeckPlugin
    ${GAZEBO_LIBRARIES}
)

add_library(DreamerRightHandPlugin
    gazebo_plugins/src/DreamerRightHandPlugin.cpp
)

target_link_libraries(DreamerRightHandPlugin
    ${GAZEBO_LIBRARIES}
)