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


/*!
 * Implements PID controllers for Dreamer's neck joints.
 */

#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

#define TOLERANCE 1e-6
#define NUM_JOINTS 4

namespace gazebo {

double DEFAULT_KP_GAINS[4] = {100, 100, 100, 100};
double DEFAULT_KD_GAINS[4] = {20, 20, 20, 20};

struct JointControllerParameters {
    physics::JointPtr joint;
    double kp;
    double kd;
    double goalPos;
    double goalVel;
};

class DreamerNeckPlugin : public ModelPlugin
{

public:
    DreamerNeckPlugin() :
        initialized(false),
        node(nullptr),
        NECK_JOINT_NAMES(nullptr)
    {
        // Start up ROS
        std::string name = "DreamerNeckPlugin";
        int argc = 0;
        ros::init(argc, NULL, name);

        // Define the joints to be controlled
        NECK_JOINT_NAMES = new std::string[NUM_JOINTS]
        {
            "lower_neck_pitch",
            "upper_neck_yaw",
            "upper_neck_roll",
            "upper_neck_pitch"
        };
    }

    ~DreamerNeckPlugin()
    {
        if (node != nullptr) delete node;
        if (NECK_JOINT_NAMES != nullptr) delete[] NECK_JOINT_NAMES;
    }

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        // Store the pointer to the model
        this->model = _parent;

        // Set up parameters for this plugin
        for (size_t ii = 0; ii < NUM_JOINTS; ii++)
        {
            physics::JointPtr joint = this->model->GetJoint(NECK_JOINT_NAMES[ii]);

            if (joint == nullptr)
            {
                gzerr << "Failed to find joint " << NECK_JOINT_NAMES[ii] << ", plugin will be disabled." << std::endl;
                return;
            }

            struct JointControllerParameters params;
            params.joint =  joint;
            params.kp = DEFAULT_KP_GAINS[ii];
            params.kd = DEFAULT_KD_GAINS[ii];
            params.goalPos = 0;
            params.goalVel = 0;

            controllerParams.push_back(params);
        }

        // ROS Nodehandle
        this->node = new ros::NodeHandle();

        // Listeners for published updates to embedded controller
        goalPosSubscriber = this->node->subscribe("neck/goal_position", 1000, &DreamerNeckPlugin::goalPositionCallback, this);
        goalVelSubscriber = this->node->subscribe("neck/goal_velocity", 1000, &DreamerNeckPlugin::goalVelocityCallback, this);
        kpSubscriber = this->node->subscribe("neck/kp", 1000, &DreamerNeckPlugin::kpCallback, this);
        kdSubscriber = this->node->subscribe("neck/kd", 1000, &DreamerNeckPlugin::kdCallback, this);

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&DreamerNeckPlugin::onUpdate, this));

        // Instantiate a ROS topic publishers
        currPosPublisher = this->node->advertise<std_msgs::Float64MultiArray>("neck/current_position", 1, false);
        currVelPublisher = this->node->advertise<std_msgs::Float64MultiArray>("neck/current_velocity", 1, false);
        goalPosPublisher = this->node->advertise<std_msgs::Float64MultiArray>("neck/goal_position", 1, false);
        goalVelPublisher = this->node->advertise<std_msgs::Float64MultiArray>("neck/goal_velocity", 1, false);
        posErrorPublisher = this->node->advertise<std_msgs::Float64MultiArray>("neck/position_error", 1, false);
        velErrorPublisher = this->node->advertise<std_msgs::Float64MultiArray>("neck/velocity_error", 1, false);
        torquePublisher = this->node->advertise<std_msgs::Float64MultiArray>("neck/torque_cmd", 1, false);

        currPosMsg.layout.dim.resize(1);
        currPosMsg.layout.dim[0].stride = NUM_JOINTS;
        currPosMsg.layout.dim[0].size = NUM_JOINTS;
        currPosMsg.data.resize(NUM_JOINTS);

        currVelMsg.layout.dim.resize(1);
        currVelMsg.layout.dim[0].stride = NUM_JOINTS;
        currVelMsg.layout.dim[0].size = NUM_JOINTS;
        currVelMsg.data.resize(NUM_JOINTS);

        goalPosMsg.layout.dim.resize(1);
        goalPosMsg.layout.dim[0].stride = NUM_JOINTS;
        goalPosMsg.layout.dim[0].size = NUM_JOINTS;
        goalPosMsg.data.resize(NUM_JOINTS);

        goalVelMsg.layout.dim.resize(1);
        goalVelMsg.layout.dim[0].stride = NUM_JOINTS;
        goalVelMsg.layout.dim[0].size = NUM_JOINTS;
        goalVelMsg.data.resize(NUM_JOINTS);

        posErrorMsg.layout.dim.resize(1);
        posErrorMsg.layout.dim[0].stride = NUM_JOINTS;
        posErrorMsg.layout.dim[0].size = NUM_JOINTS;
        posErrorMsg.data.resize(NUM_JOINTS);

        velErrorMsg.layout.dim.resize(1);
        velErrorMsg.layout.dim[0].stride = NUM_JOINTS;
        velErrorMsg.layout.dim[0].size = NUM_JOINTS;
        velErrorMsg.data.resize(NUM_JOINTS);

        torqueMsg.layout.dim.resize(1);
        torqueMsg.layout.dim[0].stride = NUM_JOINTS;
        torqueMsg.layout.dim[0].size = NUM_JOINTS;
        torqueMsg.data.resize(NUM_JOINTS);

        initialized = true;
    }

    void goalPositionCallback(const std_msgs::Float64MultiArray goalMsg)
    {
        std::stringstream msgBuff;
        msgBuff << "Setting goal positions to be:";

        for (size_t ii = 0; ii < NUM_JOINTS; ii++)
        {
            msgBuff << "\n  - " << controllerParams[ii].joint->GetName() << ": " << goalMsg.data[ii];
            controllerParams[ii].goalPos = goalMsg.data[ii];
        }

        gzdbg << msgBuff.str() << std::endl;
    }

    void goalVelocityCallback(const std_msgs::Float64MultiArray goalMsg)
    {
        std::stringstream msgBuff;
        msgBuff << "Setting goal velocities to be:";

        for (size_t ii = 0; ii < NUM_JOINTS; ii++)
        {
            msgBuff << "\n  - " << controllerParams[ii].joint->GetName() << ": " << goalMsg.data[ii];
            controllerParams[ii].goalVel = goalMsg.data[ii];
        }

        gzdbg << msgBuff.str() << std::endl;
    }

    void kpCallback(const std_msgs::Float64MultiArray goalMsg)
    {
        std::stringstream msgBuff;
        msgBuff << "Setting Kp to be:";

        for (size_t ii = 0; ii < NUM_JOINTS; ii++)
        {
            msgBuff << "\n  - " << controllerParams[ii].joint->GetName() << ": " << goalMsg.data[ii];
            controllerParams[ii].kp = goalMsg.data[ii];
        }

        gzdbg << msgBuff.str() << std::endl;
    }

    void kdCallback(const std_msgs::Float64MultiArray goalMsg)
    {
        std::stringstream msgBuff;
        msgBuff << "Setting Kd to be:";

        for (size_t ii = 0; ii < NUM_JOINTS; ii++)
        {
            msgBuff << "\n  - " << controllerParams[ii].joint->GetName() << ": " << goalMsg.data[ii];
            controllerParams[ii].kd = goalMsg.data[ii];
        }

        gzdbg << msgBuff.str() << std::endl;
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
        for(size_t ii = 0; ii < NUM_JOINTS; ii++)
        {
            physics::JointPtr joint = controllerParams[ii].joint;

            double currPos = joint->GetAngle(0).Radian();
            double currVel = joint->GetVelocity(0);

            double errPos = controllerParams[ii].goalPos - currPos;
            double errVel = controllerParams[ii].goalVel - currVel;

            double torque = controllerParams[ii].kp * errPos + controllerParams[ii].kd * errVel;

            joint->SetForce(0, torque);

            currPosMsg.data[ii] = currPos;
            currVelMsg.data[ii] = currVel;
            goalPosMsg.data[ii] = controllerParams[ii].goalPos;
            goalVelMsg.data[ii] = controllerParams[ii].goalVel;
            posErrorMsg.data[ii] = errPos;
            velErrorMsg.data[ii] = errVel;
            torqueMsg.data[ii] = torque;
        }

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

    // ROS Publishers
    ros::Publisher currPosPublisher, currVelPublisher,
                   goalPosPublisher, goalVelPublisher,
                   posErrorPublisher, velErrorPublisher,
                   torquePublisher;

    // Messages to be published
    std_msgs::Float64MultiArray currPosMsg, currVelMsg,
                                goalPosMsg, goalVelMsg,
                                posErrorMsg, velErrorMsg,
                                torqueMsg;

    std::vector<struct JointControllerParameters> controllerParams;

    // Subscribers to ros topics for hand controls
    ros::Subscriber goalPosSubscriber;
    ros::Subscriber goalVelSubscriber;
    ros::Subscriber kpSubscriber;
    ros::Subscriber kdSubscriber;

    // Hard code the joint names.  TODO: Obtain this via a ROS service call to the controller.
    std::string * NECK_JOINT_NAMES;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(DreamerNeckPlugin)

} // namespace gazebo