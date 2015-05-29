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
 * This interpolates a quaternion between two quaternions.
 * Publishes the interpolated quaternion onto the ROS topic that is
 * subscribed to by ControlIt!.
 *
 * It uses Eigen's QuaternionBase.slerp(...) method:
 * http://eigen.tuxfamily.org/dox/classEigen_1_1QuaternionBase.html#aa29a81b780c3012d0fd126a4525781c2
 */

#include <ros/ros.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

#include <dreamer_controlit_common/QuatInterpMsg.h>
#include <controlit/addons/eigen/LinearAlgebra.hpp>

using controlit::addons::eigen::Vector;
using controlit::addons::eigen::Matrix;
using controlit::addons::eigen::Quaternion;

ros::Publisher publisherR, publisherL; // right and left hand publishers

std_msgs::Float64MultiArray goalMsgR, goalMsgL; // messages for publishing

Quaternion startQuatR, startQuatL, endQuatR, endQuatL, interpQuatR, interpQuatL;

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

void updateOrientationCallback(const boost::shared_ptr<dreamer_controlit_common::QuatInterpMsg const> & msgPtr)
{
    startQuatR.w() = msgPtr->rhStart.w;
    startQuatR.x() = msgPtr->rhStart.x;
    startQuatR.y() = msgPtr->rhStart.y;
    startQuatR.z() = msgPtr->rhStart.z;

    startQuatL.w() = msgPtr->lhStart.w;
    startQuatL.x() = msgPtr->lhStart.x;
    startQuatL.y() = msgPtr->lhStart.y;
    startQuatL.z() = msgPtr->lhStart.z;

    endQuatR.w() = msgPtr->rhEnd.w;
    endQuatR.x() = msgPtr->rhEnd.x;
    endQuatR.y() = msgPtr->rhEnd.y;
    endQuatR.z() = msgPtr->rhEnd.z;

    endQuatL.w() = msgPtr->lhEnd.w;
    endQuatL.x() = msgPtr->lhEnd.x;
    endQuatL.y() = msgPtr->lhEnd.y;
    endQuatL.z() = msgPtr->lhEnd.z;

    interpQuatR = startQuatR.slerp(msgPtr->rhProportion, endQuatR);
    interpQuatL = startQuatL.slerp(msgPtr->lhProportion, endQuatL);

    ROS_INFO_STREAM("Doing update:\n"
        << "  - right wrist start: " << startQuatR << "\n"
        << "  - right write end: " << endQuatR << "\n"
        << "  - left wrist start: " << startQuatL << "\n"
        << "  - left wrist end: " << endQuatL << "\n"
        << "  - right proportion: " << msgPtr->rhProportion << "\n"
        << "  - left proportion: " << msgPtr->lhProportion << "\n"
        << "  - right interpolated orientation: " << interpQuatR << "\n"
        << "  - left interpolated orientation: " << interpQuatL);

    goalMsgR.data[0] = interpQuatR.w();
    goalMsgR.data[1] = interpQuatR.x();
    goalMsgR.data[2] = interpQuatR.y();
    goalMsgR.data[3] = interpQuatR.z();

    goalMsgL.data[0] = interpQuatL.w();
    goalMsgL.data[1] = interpQuatL.x();
    goalMsgL.data[2] = interpQuatL.y();
    goalMsgL.data[3] = interpQuatL.z();

    publisherR.publish(goalMsgR);
    publisherL.publish(goalMsgL);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_quaterinon_trajectory");

    ros::NodeHandle nh;

    // Create publishers for the new position and quaternion goals
    publisherR = nh.advertise<std_msgs::Float64MultiArray>("/dreamer_controller/RightHandOrientation/goalOrientation", 1000);
    publisherL = nh.advertise<std_msgs::Float64MultiArray>("/dreamer_controller/LeftHandOrientation/goalOrientation", 1000);

    // Define the messages to be published
    std_msgs::MultiArrayDimension dimMsg;

    dimMsg.size = 4;
    dimMsg.stride = 4;

    goalMsgR.layout.data_offset = 0;
    goalMsgR.layout.dim.push_back(dimMsg);
    goalMsgR.data.resize(4);

    goalMsgL.layout.data_offset = 0;
    goalMsgL.layout.dim.push_back(dimMsg);
    goalMsgL.data.resize(4);

    // Create the subscriber
    ros::Subscriber subscriber = nh.subscribe("/dreamer_controller/trajectory_generator/update_orientation", 1000, updateOrientationCallback);

    ROS_INFO_STREAM("Waiting for trajectory generator's publisher to connect...");

    ros::Rate loop_rate(10);
    while (subscriber.getNumPublishers() == 0)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO_STREAM("Trajectory generator connected, starting to accept update commands...");

    // start the ROS main loop
    ros::spin();
}