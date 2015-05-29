#!/bin/bash

echo Creating directory \'urdf\'
mkdir -p hume/urdf

echo Generating hume/urdf/hume.urdf
rosrun xacro xacro.py hume/xacro/hume.xacro -o hume/urdf/hume.urdf

echo Generating hume/urdf/hume_with_gazebo_plugins.urdf
rosrun xacro xacro.py hume/xacro/hume_with_gazebo_plugins.xacro -o hume/urdf/hume_with_gazebo_plugins.urdf

# echo Generating hume/urdf/hume_rapid.urdf
# rosrun xacro xacro.py hume/xacro/hume_rapid.xacro -o hume/urdf/hume_rapid.urdf

echo Done!