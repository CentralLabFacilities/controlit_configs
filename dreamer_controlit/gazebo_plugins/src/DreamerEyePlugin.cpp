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

#include <boost/bind.hpp>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <map>
#include <stdexcept>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/Angle.hh>

#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Header.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"

#include <XmlRpcValue.h>
#include <XmlRpcException.h>

#define TOLERANCE 1e-6
#define NUM_JOINTS 3

namespace gazebo {
class DreamerEyePlugin : public ModelPlugin
{

public:
    DreamerEyePlugin() :
        initialized(false),
        node(nullptr),
        EYE_JOINT_NAMES(nullptr)
    {
        // Start up ROS
        std::string name = "DreamerEyePlugin";
        int argc = 0;
        ros::init(argc, NULL, name);

        lastPubTime = ros::Time::now();

        // Define the joints to be controlled
        EYE_JOINT_NAMES = new std::string[NUM_JOINTS]
        {
            "eye_pitch",
            "right_eye_yaw",
            "left_eye_yaw"
        };
    }

    ~DreamerEyePlugin()
    {
        if (node != nullptr) delete node;
        if (EYE_JOINT_NAMES != nullptr) delete[] EYE_JOINT_NAMES;
    }

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        // Store the pointer to the model
        this->model = _parent;
        physics::Joint_V allJoints = this->model->GetJoints();
        std::vector<std::string> joint_names;

        for (size_t ii = 0; ii < allJoints.size(); ii++)
        {
            // Pick out non-fixed joints
            if(allJoints[ii]->GetLowerLimit(0).Radian() < -TOLERANCE && allJoints[ii]->GetUpperLimit(0).Radian() > TOLERANCE)
            {
                joints.insert(make_pair(allJoints[ii]->GetName(), allJoints[ii]));
                joint_names.push_back(allJoints[ii]->GetName());
                //gzprint<<"Joint "<<allJoints[ii]->GetName()<<" is movable."<<std::endl;
            }
        }

        // TODO : CHECK base joints are in all joints!!!
        // TODO : Get KP/KD from parameters at startup!!!

        // Set up parameters for this plugin
        for (size_t ii = 0; ii < NUM_JOINTS; ii++)
        {
            physics::JointPtr _jptr = this->model->GetJoint(EYE_JOINT_NAMES[ii]);
            if (_jptr == nullptr)
            {
                gzerr << "Failed to find joint " << EYE_JOINT_NAMES[ii] << ", plugin will be disabled." << std::endl;
                return;
            }
            eye_joints.insert(std::make_pair(EYE_JOINT_NAMES[ii], _jptr));
            kp.insert(std::make_pair(EYE_JOINT_NAMES[ii], 0.01));
            kd.insert(std::make_pair(EYE_JOINT_NAMES[ii], 0.001));
            goalPosition.insert(std::make_pair(EYE_JOINT_NAMES[ii],0.0));
        }

        // ROS Nodehandle
        this->node = new ros::NodeHandle;

        // Listeners for published updates to embedded controller
        this->jointPositionGoalSub = this->node->subscribe("eye_goal_position", 1000, &DreamerEyePlugin::GoalJointPositionCallback, this);


        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&DreamerEyePlugin::onUpdate, this));

        initialized = true;
    }

    void GoalJointPositionCallback(const std_msgs::Float64MultiArray goal_msg)
    {
        int ii = 0;
        for (auto & jnt : eye_joints)
        {
          goalPosition[jnt.first] = goal_msg.data[ii++];
          gzdbg << "setting joint " << jnt.first << " to " << goalPosition[jnt.first] << std::endl;
        }
    }

    /*!
     * Periodically called by Gazebo during each cycle of the simulation.
     */
    void onUpdate()
    {
        // Process pending ROS messages, etc.
        ros::spinOnce();

        // Abort if we failed to initialize
        if (!initialized) return;

        // Update the torque commands and fill in the joint state message
        for(auto & jnt : eye_joints)
        {
            physics::JointPtr camera_joint_ptr = eye_joints[jnt.first];
            double angle = camera_joint_ptr->GetAngle(0).Radian();
            double velocity = camera_joint_ptr->GetVelocity(0);
            double torque = kp[jnt.first] * (goalPosition[jnt.first] - angle) - kd[jnt.first] * velocity;
            camera_joint_ptr->SetForce(0,torque);
        }
    }

private:

    // Whether this plugin successfully initialized.  If this is false, this plugin will disable itself.
    bool initialized;

    // Pointer to the model
    physics::ModelPtr model;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    // ROS Nodehandle
    ros::NodeHandle * node;

    // ROS Subscriber for JointState messages
    ros::Subscriber subJointState;

    // ROS subscriber for Float64MultiArray messages
    // ros::Subscriber subMultiArray;

    // ROS Publisher
    ros::Publisher pub;

    // Store joint pointers in a map indexed by the joint name interpretted from the URDF file


    // Stores a pointer to each joint
    std::map<std::string, physics::JointPtr> joints;
    std::map<std::string, physics::JointPtr> eye_joints;

    // Embedded control parameters (for base joints)
    std::map<std::string, double> kp;
    std::map<std::string, double> kd;
    std::map<std::string, double> goalPosition;

    // Subscribers to ros topics for hand controls
    ros::Subscriber jointPositionGoalSub;

    // The time when the robot's state was last published
    ros::Time lastPubTime;

    // Hard code the joint names.  TODO: Obtain this via a ROS service call to the controller.
    std::string * EYE_JOINT_NAMES;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(DreamerEyePlugin)

} // namespace gazebo