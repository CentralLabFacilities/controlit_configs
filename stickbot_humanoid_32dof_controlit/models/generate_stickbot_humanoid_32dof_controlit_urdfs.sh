#!/bin/bash

echo Creating directory \'urdf\'
mkdir -p stickbot_humanoid_32dof_controlit/urdf

echo Generating stickbot_humanoid_32dof_controlit/urdf/stickbot_humanoid_32dof.urdf
rosrun xacro xacro.py stickbot_humanoid_32dof_controlit/xacro/stickbot_humanoid_32dof.xacro -o stickbot_humanoid_32dof_controlit/urdf/stickbot_humanoid_32dof.urdf

echo Done!