#!/bin/bash

echo Creating directory \'urdf\'
mkdir -p trikey_controlit/urdf

echo Generating trikey_controlit/urdf/trikey_controlit.urdf
rosrun xacro xacro.py trikey_controlit/xacro/trikey_controlit.xacro -o trikey_controlit/urdf/trikey_controlit.urdf

echo Generating trikey_controlit/urdf/trikey_gazebo.urdf
rosrun xacro xacro.py trikey_controlit/xacro/trikey_gazebo.xacro -o trikey_controlit/urdf/trikey_gazebo.urdf

echo Done!