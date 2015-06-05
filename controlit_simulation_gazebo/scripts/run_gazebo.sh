#!/bin/sh

# Argument should be a fully-qualified path to a .world file
gazebo -s libgazebo_ros_api_plugin.so --verbose -u $1