#!/bin/bash

echo Creating directory \'urdf\'
mkdir -p atlas_plain_controlit/urdf

echo Generating atlas_plain_controlit/urdf/atlas_plain_with_sandia_hands_gazebo.urdf
rosrun xacro xacro.py atlas_plain_controlit/xacro/atlas_plain_with_sandia_hands_gazebo.xacro -o atlas_plain_controlit/urdf/atlas_plain_with_sandia_hands_gazebo.urdf

echo Generating atlas_plain_controlit/urdf/atlas_plain_with_sandia_hands_controlit.urdf
rosrun xacro xacro.py atlas_plain_controlit/xacro/atlas_plain_with_sandia_hands_controlit.xacro -o atlas_plain_controlit/urdf/atlas_plain_with_sandia_hands_controlit.urdf

echo Done!