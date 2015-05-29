#!/bin/bash

echo Creating directory \'urdf\'
mkdir -p dreamer_testbed_controlit/urdf

echo Generating dreamer_testbed_controlit/urdf/dreamer_testbed_gazebo.urdf
rosrun xacro xacro.py dreamer_testbed_controlit/xacro/dreamer_testbed_gazebo.xacro -o dreamer_testbed_controlit/urdf/dreamer_testbed_gazebo.urdf

echo Done!