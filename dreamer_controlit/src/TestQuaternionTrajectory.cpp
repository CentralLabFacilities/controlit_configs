/*
 * Copyright (C) 2015 The University of Texas at Austin and the
 * Institute of Human Machine Cognition. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 2.1 of
 * the License, or (at your option) any later version. See
 * <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

/**
 * This is a test program that generates a trajectory between the 
 * following two quaternions defined in the world coordinate frame:
 *
 * [0, 1, 0, 0]
 * [0.00123728, 0.887812, 0.00056107, 0.460204]
 *
 * The quaternion elements are ordered as follows: [w, x, y, z].
 *
 * It uses Eigen's QuaternionBase.slerp(...) method:
 * http://eigen.tuxfamily.org/dox/classEigen_1_1QuaternionBase.html#aa29a81b780c3012d0fd126a4525781c2
 */

#include <ros/ros.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

#include <controlit/addons/eigen/LinearAlgebra.hpp>

using controlit::addons::eigen::Vector;
using controlit::addons::eigen::Matrix;
// using controlit::addons::eigen::Vector3d;
// using controlit::addons::eigen::Matrix3d;
using controlit::addons::eigen::Quaternion;

ros::Publisher publisher;

/** 
 * Override the '<<' operator for printing an quaternion to an output stream.
 */
std::ostream & operator<<(std::ostream & os, const Quaternion & qq)
{
    // std::stringstream ss;
    os << "(" << qq.w() << ", " << qq.x() << ", " << qq.y() << ", " << qq.z() << ")";
    // return ss.str();
    return os;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_quaterinon_trajectory");

    ros::NodeHandle nh;

    // Create publishers for the new position and quaternion goals
    publisher = nh.advertise<std_msgs::Float64MultiArray>("/dreamer_controller/RightHandOrientation/goalOrientation", 1000);
  
    // Define the message to be published
    std_msgs::MultiArrayDimension dimMsg;
    std_msgs::Float64MultiArray goalMsg;

    dimMsg.size = 4;
    dimMsg.stride = 4;

    goalMsg.layout.data_offset = 0;
    goalMsg.layout.dim.push_back(dimMsg);
    goalMsg.data.resize(4);

    // Define the start and end quaternions
    Quaternion startQuat(0, 1, 0, 0);
    Quaternion endQuat(0.00123728, 0.887812, 0.00056107, 0.460204);

    ROS_INFO_STREAM("Start: " << startQuat);
    ROS_INFO_STREAM("End: " << endQuat);

    // Define an intermediate quaternion
    Quaternion interQuat;

    // start the ROS main loop
    ros::Rate loop_rate(10);
    for (double tt = 0; tt < 1.0 && ros::ok(); tt += 0.01)  // tt is a proportional time factor
    {
        ros::spinOnce();

        interQuat = startQuat.slerp(tt, endQuat);

        ROS_INFO_STREAM("Intermediate quaternion at time " << tt << ": " << interQuat);

        goalMsg.data[0] = interQuat.w();
        goalMsg.data[1] = interQuat.x();
        goalMsg.data[2] = interQuat.y();
        goalMsg.data[3] = interQuat.z();

        publisher.publish(goalMsg);

        loop_rate.sleep();
    }
}