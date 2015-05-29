#!/bin/bash

#rostopic pub -1 /camera_goal_position std_msgs/Float64MultiArray '{layout: {dim: [{label: "x", size: 4, stride: 4}], data_offset: 0}, data: [0.0, 1.57, 0.0, 1.57]}'

rostopic pub -1 /camera_goal_position std_msgs/Float64MultiArray '{layout: {dim: [{label: "x", size: 4, stride: 4}], data_offset: 0}, data: [-0.15, 1.0, 0.15, 1.0]}'
