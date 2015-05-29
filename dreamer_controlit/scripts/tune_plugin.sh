#!/bin/bash

rostopic pub -1 /dreamer_slave_joint_kp std_msgs/Float64 100

rostopic pub -1 /dreamer_slave_joint_kd std_msgs/Float64 30

rostopic pub -1 /dreamer_no_left_arm_controller/RightHandPosition/goalPosition std_msgs/Float64MultiArray '{layout: {dim: [{label: "x", size: 3, stride: 3}], data_offset: 0}, data: [0.1, -0.3, 0.1]}'
rostopic pub -1 /dreamer_no_left_arm_controller/JPosTask/goalPosition std_msgs/Float64MultiArray '{layout: {dim: [{label: "x", size: 10, stride: 10}], data_offset: 0}, data: [0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'





rostopic pub -1 /dreamer_no_left_arm_controller/RightHandPosition/goalPosition std_msgs/Float64MultiArray '{layout: {dim: [{label: "x", size: 3, stride: 3}], data_offset: 0}, data: [-0.2, -0.1, 0.15]}'

rostopic pub -1 /dreamer_no_left_arm_controller/RightFingerHeading/goalVector std_msgs/Float64MultiArray '{layout: {dim: [{label: "x", size: 3, stride: 3}], data_offset: 0}, data: [0, 0, -1.0]}'

rostopic pub -1 /dreamer_no_left_arm_controller/RightPalmHeading/goalVector std_msgs/Float64MultiArray '{layout: {dim: [{label: "x", size: 3, stride: 3}], data_offset: 0}, data: [0, 1.0, 1.0]}'


rostopic pub -1 /dreamer_no_left_arm_controller/RightFingerHeading/goalVector std_msgs/Float64MultiArray '{layout: {dim: [{label: "x", size: 3, stride: 3}], data_offset: 0}, data: [1.0, 0, -1.0]}'
