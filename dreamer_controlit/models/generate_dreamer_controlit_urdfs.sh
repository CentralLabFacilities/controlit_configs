#!/bin/bash

echo Creating directory \'urdf\'
mkdir -p dreamer_controlit/urdf

echo Generating dreamer_controlit/urdf/dreamer_controlit_gazebo.urdf
rosrun xacro xacro.py dreamer_controlit/xacro/dreamer_controlit_gazebo.xacro -o dreamer_controlit/urdf/dreamer_controlit_gazebo.urdf

echo Generating dreamer_controlit/urdf/dreamer_controlit_rviz.urdf
rosrun xacro xacro.py dreamer_controlit/xacro/dreamer_controlit_rviz.xacro -o dreamer_controlit/urdf/dreamer_controlit_rviz.urdf

echo Done!