#!/bin/bash

echo Creating directory \'urdf\'
mkdir -p trikey_pinned_controlit/urdf

echo Generating trikey_pinned_controlit/urdf/trikey_pinned_gazebo.urdf
rosrun xacro xacro.py trikey_pinned_controlit/xacro/trikey_pinned_gazebo.xacro -o trikey_pinned_controlit/urdf/trikey_pinned_gazebo.urdf

echo Generating trikey_pinned_controlit/urdf/trikey_pinned_controlit.urdf
rosrun xacro xacro.py trikey_pinned_controlit/xacro/trikey_pinned_controlit.xacro -o trikey_pinned_controlit/urdf/trikey_pinned_controlit.urdf

echo Done!