#!/bin/bash

echo Creating directory \'urdf\'
mkdir -p atlas_plain_pinned_controlit/urdf

echo Generating atlas_plain_pinned_controlit/urdf/atlas_plain_pinned_gazebo.urdf
rosrun xacro xacro.py atlas_plain_pinned_controlit/xacro/atlas_plain_pinned_gazebo.xacro -o atlas_plain_pinned_controlit/urdf/atlas_plain_pinned_gazebo.urdf

echo Done!