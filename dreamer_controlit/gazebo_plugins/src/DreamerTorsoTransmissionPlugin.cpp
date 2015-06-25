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

#include <XmlRpcValue.h>
#include <XmlRpcException.h>

namespace gazebo {

#define DEFAULT_KP 1000
#define DEFAULT_KD 50

class DreamerTorsoTransmissionPlugin : public ModelPlugin
{

    public: DreamerTorsoTransmissionPlugin() :
        initialized(false),
        node(nullptr),
        slaveJnt(nullptr),
        masterJnt(nullptr)
    {
        // Start up ROS
        std::string name = "DreamerTorsoTransmissionPlugin";
        int argc = 0;
        ros::init(argc, NULL, name);

        lastPubTime = ros::Time::now();

        MASTER_JOINT_NAME = "torso_lower_pitch";
        SLAVE_JOINT_NAME = "torso_upper_pitch";
    }

    public: ~DreamerTorsoTransmissionPlugin()
    {
        if (node != nullptr) delete this->node;
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        // Store the pointer to the model
        this->model = _parent;

        // TODO : CHECK base joints are in all joints!!!
        slaveJnt = this->model->GetJoint(SLAVE_JOINT_NAME);
        masterJnt = this->model->GetJoint(MASTER_JOINT_NAME);

        if (slaveJnt == nullptr)
        {
            gzerr << "Failed to find slave joint " << SLAVE_JOINT_NAME << ", plugin will be disabled." << std::endl;
            return;
        }

        if (masterJnt == nullptr)
        {
            gzerr << "Failed to find master joint " << MASTER_JOINT_NAME << ", plugin will be disabled." << std::endl;
            return;
        }

        kp = DEFAULT_KP;
        kd = DEFAULT_KD;

        // ROS Nodehandle
        this->node = new ros::NodeHandle;

        // Listeners for published updates to embedded controller
        this->kpSub = this->node->subscribe("torso/kp", 1000, &DreamerTorsoTransmissionPlugin::KpCallback, this);
        this->kdSub = this->node->subscribe("torso/kd", 1000, &DreamerTorsoTransmissionPlugin::KdCallback, this);

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&DreamerTorsoTransmissionPlugin::onUpdate, this));

        // Instantiate a ROS topic publishers
        currPosPublisher = this->node->advertise<std_msgs::Float64>("torso/current_position", 1, false);
        currVelPublisher = this->node->advertise<std_msgs::Float64>("torso/current_velocity", 1, false);
        goalPosPublisher = this->node->advertise<std_msgs::Float64>("torso/goal_position", 1, false);
        goalVelPublisher = this->node->advertise<std_msgs::Float64>("torso/goal_velocity", 1, false);
        posErrorPublisher = this->node->advertise<std_msgs::Float64>("torso/position_error", 1, false);
        velErrorPublisher = this->node->advertise<std_msgs::Float64>("torso/velocity_error", 1, false);
        torquePublisher = this->node->advertise<std_msgs::Float64>("torso/torque_cmd", 1, false);

        initialized = true;
    }

    public: void KpCallback(const std_msgs::Float64 gain_msg)
    {
        kp = gain_msg.data;
        gzdbg << "slave joint Kp callback\n";
    }

    public: void KdCallback(const std_msgs::Float64 gain_msg)
    {
        kd = gain_msg.data;
        gzdbg << "slave joint Kd callback\n";
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

        double masterAngle = masterJnt->GetAngle(0).Radian();
        double masterVelocity = masterJnt->GetVelocity(0);

        double slaveAngle = slaveJnt->GetAngle(0).Radian();
        double slaveVelocity = slaveJnt->GetVelocity(0);

        double errPos = masterAngle - slaveAngle;
        double errVel = masterVelocity - slaveVelocity;

        double slaveTorque = kp * errPos + kd * errVel;

        slaveJnt->SetForce(0, slaveTorque);

        currPosMsg.data = slaveAngle;
        currVelMsg.data = slaveVelocity;
        goalPosMsg.data = masterAngle;
        goalVelMsg.data = masterVelocity;
        posErrorMsg.data = errPos;
        velErrorMsg.data = errVel;
        torqueMsg.data = slaveTorque;

        currPosPublisher.publish(currPosMsg);
        currVelPublisher.publish(currVelMsg);
        goalPosPublisher.publish(goalPosMsg);
        goalVelPublisher.publish(goalVelMsg);
        posErrorPublisher.publish(posErrorMsg);
        velErrorPublisher.publish(velErrorMsg);
        torquePublisher.publish(torqueMsg);
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

    // Embedded control parameters...PD loop mimics drive train
    double kp;
    double kd;

    // Subscribers to ros topics for hand controls
    ros::Subscriber kpSub;
    ros::Subscriber kdSub;

    // The time when the robot's state was last published
    ros::Time lastPubTime;

    // Hard code the joint names.  TODO: Obtain this via a ROS service call to the controller.
    std::string SLAVE_JOINT_NAME;
    std::string MASTER_JOINT_NAME;

    physics::JointPtr slaveJnt;
    physics::JointPtr masterJnt;

    // ROS Publishers
    ros::Publisher currPosPublisher, currVelPublisher,
                   goalPosPublisher, goalVelPublisher,
                   posErrorPublisher, velErrorPublisher,
                   torquePublisher;

    // Messages to be published
    std_msgs::Float64 currPosMsg, currVelMsg,
                      goalPosMsg, goalVelMsg,
                      posErrorMsg, velErrorMsg,
                      torqueMsg;


};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(DreamerTorsoTransmissionPlugin)

} //  namespace gazebo