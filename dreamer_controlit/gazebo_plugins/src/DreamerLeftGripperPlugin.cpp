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

#include "std_srvs/Empty.h"

#include <XmlRpcValue.h>
#include <XmlRpcException.h>

namespace gazebo {

class DreamerLeftGripperPlugin : public ModelPlugin
{

    public: DreamerLeftGripperPlugin() :
        initialized(false),
        node(nullptr)
    {
        // Start up ROS
        std::string name = "DreamerLeftGripperPlugin";
        int argc = 0;
        ros::init(argc, NULL, name);

        lastPubTime = ros::Time::now();

        // MASTER_JOINT_NAME = "l_gripper_l_finger_joint";
        SLAVE_JOINT_NAME_Vector.push_back("l_gripper_l_finger_joint");
        SLAVE_JOINT_NAME_Vector.push_back("l_gripper_l_finger_tip_joint");
        SLAVE_JOINT_NAME_Vector.push_back("l_gripper_r_finger_joint");
        SLAVE_JOINT_NAME_Vector.push_back("l_gripper_r_finger_tip_joint");

        // TARGET: 0/0.548 (lowerlimit/upperlimit)
        target_position = 0.548;
    }

    public: ~DreamerLeftGripperPlugin()
    {
      if (node != nullptr) delete this->node;
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        // Store the pointer to the model
        this->model = _parent;

        // TODO : CHECK base joints are in all joints!!!
        for (size_t ii = 0; ii < SLAVE_JOINT_NAME_Vector.size(); ii++)
        {
            physics::JointPtr currJoint = this->model->GetJoint(SLAVE_JOINT_NAME_Vector[ii]);
            if (currJoint == nullptr)
            {
                gzerr << "Failed to find joint " << SLAVE_JOINT_NAME_Vector[ii] << ", plugin will be disabled." << std::endl;
                return;
            }
            slaveJntVector.push_back(currJoint);
        }

        masterJnt = this->model->GetJoint(MASTER_JOINT_NAME);

        //Cannot be too large
        kp = 10.0;
        kd = 2.0;

        // ROS Nodehandle
        this->node = new ros::NodeHandle;

        this->opengripperSrv = this->node->advertiseService("gripper_open", &DreamerLeftGripperPlugin::OpenGripperCallback,this);
        this->closegripperSrv = this->node->advertiseService("gripper_close", &DreamerLeftGripperPlugin::CloseGripperCallback,this);

        // Listen to the update event. This event is broadcast every simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&DreamerLeftGripperPlugin::onUpdate, this));

        initialized = true;
    }


    public: bool OpenGripperCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
    {
      target_position=0.548;
      ROS_INFO("Gripper Open! ");
      return true;
    }

    public: bool CloseGripperCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
    {
      target_position=0;
      ROS_INFO("Gripper Close! ");
      return true;
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

        for (size_t ii = 0; ii < slaveJntVector.size(); ii++)
        {
            double slaveAngle = slaveJntVector[ii]->GetAngle(0).Radian();
            double slaveVelocity = slaveJntVector[ii]->GetVelocity(0);
            double slaveTorque = kp * (target_position - slaveAngle) + kd * (0 - slaveVelocity);
            slaveJntVector[ii]->SetForce(0, slaveTorque);
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

    // Embedded control parameters...PD loop mimics drive train
    double kp;
    double kd;
    double target_position;

    // Subscribers to ros topics for hand controls
    // ros::Subscriber gripperSub;
    ros::ServiceServer opengripperSrv;
    ros::ServiceServer closegripperSrv;

    // The time when the robot's state was last published
    ros::Time lastPubTime;

    // Hard code the joint names.  TODO: Obtain this via a ROS service call to the controller.
    std::vector<std::string> SLAVE_JOINT_NAME_Vector;
    std::string MASTER_JOINT_NAME;

    std::vector<physics::JointPtr> slaveJntVector;
    physics::JointPtr masterJnt;

};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(DreamerLeftGripperPlugin)

} //  namespace gazebo