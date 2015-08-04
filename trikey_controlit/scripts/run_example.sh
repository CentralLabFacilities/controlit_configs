#!/bin/bash

# rosparam set /trikey_controller/command_delay_ms 200
# rosparam set /trikey_controller/state_delay_ms 200

#rostopic pub -1 /trikey_controller/check_delay_parameters std_msgs/Bool true

# rostopic pub -1 /trikey_controller/embedded_kd std_msgs/Float64MultiArray '{layout: {dim: [{label: "x", size: 3, stride: 3}], data_offset: 0}, data: [0.0, 0.0, 0.0]}'

rostopic pub -1 /trikey_controller/XYTask/goalPosition std_msgs/Float64MultiArray '{layout: {dim: [{label: "x", size: 3, stride: 3}], data_offset: 0}, data: [0.5, 0.0, 0.0]}' &
rostopic pub -1 /trikey_controller/HeadingTask/goalVector std_msgs/Float64MultiArray '{layout: {dim: [{label: "x", size: 3, stride: 3}], data_offset: 0}, data: [0.0, -1.0, 0.0]}'

