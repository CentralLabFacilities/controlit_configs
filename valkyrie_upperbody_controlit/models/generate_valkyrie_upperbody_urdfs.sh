#!/bin/bash

echo Creating directory \'urdf\'
mkdir -p valkyrie_upperbody_controlit/urdf

echo Generating valkyrie_upperbody_controlit/urdf/valkyrie_upperbody_gazebo.urdf
rosrun xacro xacro.py valkyrie_upperbody_controlit/xacro/valkyrie_upperbody_gazebo.xacro -o valkyrie_upperbody_controlit/urdf/valkyrie_upperbody_gazebo.urdf

echo Generating valkyrie_upperbody_controlit/urdf/valkyrie_upperbody_controlit.urdf
rosrun xacro xacro.py valkyrie_upperbody_controlit/xacro/valkyrie_upperbody_controlit.xacro -o valkyrie_upperbody_controlit/urdf/valkyrie_upperbody_controlit.urdf

echo Done!