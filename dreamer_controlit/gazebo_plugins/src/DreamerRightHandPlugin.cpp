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
// #include <stdexcept>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
// #include <gazebo/math/Angle.hh>

#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Header.h"
// #include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"

// #include <XmlRpcValue.h>
// #include <XmlRpcException.h>

#define TOLERANCE 1e-6

namespace gazebo {

class DreamerHandPlugin : public ModelPlugin
{
    public: DreamerHandPlugin() :
        initialized(false),
        node(nullptr),
        HAND_JOINT_NAMES(nullptr)
    {
        // Start up ROS
        std::string name = "DreamerHandPlugin";
        int argc = 0;
        ros::init(argc, NULL, name);

        lastPubTime = ros::Time::now();

        HAND_JOINT_NAMES = new std::string[5]
        {
            "right_thumb_cmc",
            "right_thumb_mcp",
            "right_pointer_finger_mcp",
            "right_middle_finger_mcp",
            "right_pinky_mcp"
        };
    }

    public: ~DreamerHandPlugin()
    {
      if (node != nullptr) delete node;
      if (HAND_JOINT_NAMES != nullptr) delete[] HAND_JOINT_NAMES;
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
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
            }

        }

        // TODO : CHECK base joints are in all joints!!!
        // TODO : Get KP/KD from parameters at startup!!!

        // Set up parameters for this plugin
        for (size_t ii = 0; ii < 5; ii++)
        {
            physics::JointPtr _jptr = this->model->GetJoint(HAND_JOINT_NAMES[ii]);
            if (_jptr == nullptr)
            {
                gzerr << "DreamerRightHandPlugin: Failed to find joint " << HAND_JOINT_NAMES[ii] << ", plugin will be disabled." << std::endl;
                return;
            }
            base_joints.insert(std::make_pair(HAND_JOINT_NAMES[ii], _jptr));
            kp.insert(std::make_pair(HAND_JOINT_NAMES[ii], 1.0));
            kd.insert(std::make_pair(HAND_JOINT_NAMES[ii], 0.01));
            goalPosition.insert(std::make_pair(HAND_JOINT_NAMES[ii], 0.0));

            // map to be copied into transmisions, all 1 for now...possibly parse this from urdf?
            std::map<std::string, double> tmap;
            if(ii > 0)
            {
                auto jit = std::find(joint_names.begin(), joint_names.end(), HAND_JOINT_NAMES[ii]);
                jit++;
                if(ii == 1)
                    tmap.insert(make_pair(*jit, 0.185));
                else
                    tmap.insert(make_pair(*jit, 0.945));
                if(ii > 1)
                {
                    jit++;
                    tmap.insert(make_pair(*jit, 0.971));
                }
            }
            transmission.insert(make_pair(HAND_JOINT_NAMES[ii], tmap));
        }

        // print transmission map
        for (auto & base_jnt : base_joints)
        {
            gzdbg << "DreamerRightHandPlugin: Joint : " << base_jnt.first << " has " << transmission[base_jnt.first].size() << " slave joints \n";
            for (auto & slave_jnt : transmission[base_jnt.first])
                gzdbg << "\t " << slave_jnt.first << " factor : " << slave_jnt.second << "\n";
        }

        // ROS Nodehandle
        this->node = new ros::NodeHandle;

        // Listeners for published updates to embedded controller
        this->jointPositionGoalSub = this->node->subscribe("right_hand/goal_position", 1000, &DreamerHandPlugin::GoalJointPositionCallback, this);
        this->kpSub = this->node->subscribe("right_hand/kp", 1000, &DreamerHandPlugin::KpCallback, this);
        this->kdSub = this->node->subscribe("right_hand/kd", 1000, &DreamerHandPlugin::KdCallback, this);

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&DreamerHandPlugin::onUpdate, this));

        initialized = true;
    }

    public: void GoalJointPositionCallback(const std_msgs::Float64MultiArray goal_msg)
    {
        int ii = 0;
        for (auto & jnt : base_joints)
            goalPosition[jnt.first] = goal_msg.data[ii++];

        gzdbg << "DreamerRightHandPlugin: Right Hand Goal Position callback\n";
    }

    public: void KpCallback(const std_msgs::Float64MultiArray gain_msg)
    {
        int ii = 0;
        for (auto & jnt : base_joints)
            kp[jnt.first] = gain_msg.data[ii++];

        gzdbg << "DreamerRightHandPlugin: Right Hand Kp callback\n";
    }

    public: void KdCallback(const std_msgs::Float64MultiArray gain_msg)
    {
        int ii = 0;
        for (auto & jnt : base_joints)
            kd[jnt.first] = gain_msg.data[ii++];

        gzdbg << "DreamerRightHandPlugin: Right Kd Position callback\n";
    }

    /*!
     * Periodically called by Gazebo during each cycle of the simulation.
     */
    public: void onUpdate()
    {
        // Process pending ROS messages, etc.
        ros::spinOnce();

        // Abort if we failed to initialize
        if (!initialized) return;

        // Update the torque commands and fill in the joint state message
        for(auto & master_jnt : transmission)
        {
            physics::JointPtr master = base_joints[master_jnt.first];
            double masterAngle = master->GetAngle(0).Radian();
            double masterVelocity = master->GetVelocity(0);
            double masterTorque = kp[master_jnt.first] * (goalPosition[master_jnt.first] - masterAngle)
                                  - kd[master_jnt.first] * masterVelocity;
            master->SetForce(0, masterTorque);

            int scale = 1;
            for( auto & slave_jnt : master_jnt.second)
            {
                physics::JointPtr slave = joints[slave_jnt.first];
                double slaveAngle = slave->GetAngle(0).Radian();
                double slaveVelocity = slave->GetVelocity(0);
                double slaveTorque = kp[master_jnt.first] * (slave_jnt.second * goalPosition[master_jnt.first] - slaveAngle)
                                     + kd[master_jnt.first] * (0 * slave_jnt.second * masterVelocity - slaveVelocity);
                slave->SetForce(0, slaveTorque * pow(0.1, scale++));
            }
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

    // ROS Publisher
    ros::Publisher pub;

    // Stores a pointer to each joint
    std::map<std::string, physics::JointPtr> joints;
    std::map<std::string, physics::JointPtr> base_joints;
    std::map<std::string, std::map<std::string, double>> transmission;

    // Embedded control parameters (for base joints)
    std::map<std::string, double> kp;
    std::map<std::string, double> kd;
    std::map<std::string, double> goalPosition;

    // Subscribers to ros topics for hand controls
    ros::Subscriber kpSub;
    ros::Subscriber kdSub;
    ros::Subscriber jointPositionGoalSub;

    // The time when the robot's state was last published
    ros::Time lastPubTime;

    // Hard code the joint names.  TODO: Obtain this via a ROS service call to the controller.
    std::string * HAND_JOINT_NAMES;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(DreamerHandPlugin)

} // namespace gazebo