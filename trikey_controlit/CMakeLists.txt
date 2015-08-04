cmake_minimum_required(VERSION 2.8.3)
project(trikey_controlit)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    controlit_udp
)

find_package(gazebo REQUIRED)

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

# include (FindPkgConfig)
# if (PKG_CONFIG_FOUND)
#   pkg_check_modules(GAZEBO gazebo REQUIRED)
# else()
#   message(FATAL_ERROR "pkg-config is required; please install it")
# endif()

# message(STATUS "Found Gazebo:")
# message(STATUS " include directories: ${GAZEBO_INCLUDE_DIRS}")
# message(STATUS " libraries: ${GAZEBO_LIBRARY_DIRS}")

# include_directories(${GAZEBO_INCLUDE_DIRS})
# link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
    ${GAZEBO_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)


add_library(trikey_udp_plugin
    gazebo_plugins/src/trikey_udp_plugin.cpp
)

target_link_libraries(trikey_udp_plugin
    ${catkin_LIBRARIES}
)

add_library(trikey_udp_plugin_with_embedded_control
    gazebo_plugins/src/trikey_udp_plugin_with_embedded_control.cpp
)

target_link_libraries(trikey_udp_plugin_with_embedded_control
    ${catkin_LIBRARIES}
)

# rosbuild_add_compile_flags(trikey_udp_plugin_with_embedded_control -std=c++0x)
# rosbuild_add_library(trikey_embedded_control_udp_plugin gazebo_plugins/src/trikey_udp_plugin_with_embedded_control.cpp gazebo_plugins/src/comm_udp.c)
# rosbuild_add_compile_flags(trikey_embedded_control_udp_plugin -std=c++0x)
