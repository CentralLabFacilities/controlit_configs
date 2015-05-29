#!/bin/bash

echo Creating directory \'urdf\'
mkdir -p stickbot_lowerleg_3dof_controlit/urdf

echo Generating stickbot_lowerleg_3dof_controlit/urdf/stickbot_lowerleg_3dof_gazebo.urdf
rosrun xacro xacro.py stickbot_lowerleg_3dof_controlit/xacro/stickbot_lowerleg_3dof_gazebo.xacro -o stickbot_lowerleg_3dof_controlit/urdf/stickbot_lowerleg_3dof_gazebo.urdf

echo Done!