#!/bin/bash

echo Creating directory \'urdf\'
mkdir -p stickbot_leg_6dof_controlit/urdf

echo Generating stickbot_leg_6dof_controlit/urdf/stickbot_leg_6dof_gazebo.urdf
rosrun xacro xacro.py stickbot_leg_6dof_controlit/xacro/stickbot_leg_6dof_gazebo.xacro -o stickbot_leg_6dof_controlit/urdf/stickbot_leg_6dof_gazebo.urdf

echo Done!