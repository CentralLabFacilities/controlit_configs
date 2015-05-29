#!/bin/bash

echo Creating directory \'urdf\'
mkdir -p stickbot_upperbody_10dof_controlit/urdf

echo Generating stickbot_upperbody_10dof_controlit/urdf/stickbot_upperbody_10dof.urdf
rosrun xacro xacro.py stickbot_upperbody_10dof_controlit/xacro/stickbot_upperbody_10dof.xacro -o stickbot_upperbody_10dof_controlit/urdf/stickbot_upperbody_10dof.urdf

echo Done!