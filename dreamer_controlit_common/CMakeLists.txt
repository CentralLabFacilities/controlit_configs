cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

rosbuild_find_ros_package(controlit_cmake)
list(APPEND CMAKE_MODULE_PATH ${controlit_cmake_PACKAGE_PATH}/cmake)
include(controlitbuild)
controlit_build_init()

controlit_build_depend_eigen3()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
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

# message(STATUS ${GAZEBO_INCLUDE_DIRS})
# message(STATUS ${GAZEBO_LIBRARY_DIRS})

# include_directories(${GAZEBO_INCLUDE_DIRS})
# link_directories(${GAZEBO_LIBRARY_DIRS})

# TODO: Move the interactive markers into this package, remove it from
# dreamer_controlit and controlit_dreamer_integration.
# rosbuild_add_executable(DreamerInteractiveMarkers src/DreamerInteractiveMarkers.cpp)

rosbuild_add_executable(QuaternionInterpolator src/QuaternionInterpolator.cpp)
controlit_build_link_depends(QuaternionInterpolator)

# rosbuild_add_library(dreamer_slave_joint_plugin gazebo_plugins/src/dreamer_slave_joint_plugin.cpp)
# rosbuild_add_compile_flags(dreamer_slave_joint_plugin -std=c++0x)

# rosbuild_add_library(dreamer_eye_plugin gazebo_plugins/src/dreamer_eye_plugin.cpp)
# rosbuild_add_compile_flags(dreamer_eye_plugin -std=c++0x)