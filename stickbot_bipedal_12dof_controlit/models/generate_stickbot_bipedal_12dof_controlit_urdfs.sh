#!/bin/bash

echo Creating directory \'urdf\'
mkdir -p stickbot_bipedal_12dof_controlit/urdf

echo Generating stickbot_bipedal_12dof_controlit/urdf/stickbot_bipedal_12dof_gazebo.urdf
rosrun xacro xacro.py stickbot_bipedal_12dof_controlit/xacro/stickbot_bipedal_12dof_gazebo.xacro -o stickbot_bipedal_12dof_controlit/urdf/stickbot_bipedal_12dof_gazebo.urdf

echo Done!