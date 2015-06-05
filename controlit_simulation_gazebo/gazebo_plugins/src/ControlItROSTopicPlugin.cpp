#include <boost/bind.hpp>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <map>
#include <stdexcept>
#include <mutex>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/Angle.hh>

#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Header.h"
#include "std_msgs/Float64.h"
#include "controlit_robot_interface_library/JointState.h"

#include <XmlRpcValue.h>
#include <XmlRpcException.h>

namespace gazebo {

  class ControlItROSTopicPlugin : public ModelPlugin
  {

    public: ControlItROSTopicPlugin() :
      rttSeqNo(0)
    {

      // Start up ROS
      std::string name = "ControlIt_Plugin";
      int argc = 0;
      ros::init(argc, NULL, name);

      lastPubTime = ros::Time::now();
    }
    public: ~ControlItROSTopicPlugin()
    {
      delete this->node;
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Initialize the JointController with the model pointer
      // this->j2_controller = new physics::JointController(this->model);

      // Load parameters for this plugin
      bool paramsLoaded = this->LoadParams(_sdf);

      if (paramsLoaded)
      {
      // ROS Nodehandle
      this->node = new ros::NodeHandle;

      // For publishing the robot's current state
      this->pub = this->node->advertise<controlit_robot_interface_library::JointState>("robot_state", 1000);

      // For subscribing to the command issued by the controller
      this->sub = this->node->subscribe<controlit_robot_interface_library::JointState>("command", 1000,
        &ControlItROSTopicPlugin::commandCallback, this);

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ControlItROSTopicPlugin::onUpdate, this));
      }
    }

/*Utility function for loading joint parameters*/
    public: bool LoadParams(sdf::ElementPtr _sdf)
    {
      physics::JointPtr _jptr;
      std::string jname;

      // Get the robot namespace
      std::string robotNamespace = "";
      if (_sdf->HasElement("robotNamespace"))
      robotNamespace = _sdf->GetElement("robotNamespace")->Get<std::string>();

      // Get the robot description parameter
      std::string robotDescParam;
      robotDescParam = "robot_description";
      if (_sdf->HasElement("robotDescription"))
        robotDescParam = _sdf->GetElement("robotDescription")->Get<std::string>();
      robotDescParam = robotNamespace + "/" + robotDescParam;

      // Verify that the robot description parameter exists
      ros::NodeHandle nn;
      if(nn.hasParam(robotDescParam))
        gzdbg << "Obtaining the robot description from ROS parameter \"" << robotDescParam << "\".\n";

      // Parse the robot descrition from the robotDescParam on the server
      // This is necessary b/c the robot is loaded from a URDF, and NOT an SDF file

      std::string urdf_string;
      TiXmlDocument urdf_xml;

      if (!nn.getParam(robotDescParam, urdf_string))
      {
        gzerr << "Cannot get robot description parameter \"" << robotDescParam << "\".\n";
        throw std::runtime_error("ControlItROSTopicPlugin invalid robot description parameter \""
          + robotDescParam + "\"");
      }

      urdf_xml.Parse(urdf_string.c_str());
      TiXmlNode * urdf_root(urdf_xml.FirstChild("robot"));

      if (!urdf_root)
      {
        gzerr << "Tiny XML doc not initialized \n";
        throw std::runtime_error("ControlItROSTopicPlugin invalid robotDescParam name: robot not root \""
          +  robotDescParam + "\"");
      }

      TiXmlNode * it = urdf_root->FirstChild("joint");

      // Pick out all joint names to be stored in joints map and used to look up joint pointers.

      while(it != NULL)
      {
        jname = it->ToElement()->FirstAttribute()->ValueStr(); //the first attribute is the joint name
        gzdbg << "Parsed ["<<it->ToElement()->FirstAttribute()->ValueStr()<<"]\n";
        _jptr = this->model->GetJoint(jname);

        if(!_jptr)
        {
          std::string jtype = it->ToElement()->LastAttribute()->ValueStr();
          if(jtype != "fixed")
          {
            gzwarn << "Joint \"" << jname << "\" not found and is not fixed (type = " << jtype << ")\n";
            return false;
          }
        }
        else
        {
          joints.insert(std::make_pair(jname, _jptr));
          torques.insert(std::make_pair(jname, 0.0));
        }

        it = urdf_root->IterateChildren("joint", it);
      }

      return true;
    }

    // Called by the world update start event
    public: void onUpdate()
    {
      // Process pending ROS messages, etc.
      ros::spinOnce();

      if (mutex.try_lock())
      {
        // Create a message for holding the robot's current state
        controlit_robot_interface_library::JointState currRobotState;
        currRobotState.header.stamp = ros::Time::now();

        // Save the RTT sequence number.
        currRobotState.rtt_seq_no = rttSeqNo;

        // Update the torque commands and save the robot's current state
        std::map<std::string, physics::JointPtr>::iterator name_it;
        for(name_it = joints.begin(); name_it != joints.end(); name_it++)
        {
          std::string jointName = name_it->first;
          double effort = torques[jointName];

          // Update Gazebo with the joint torque command
          name_it->second->SetForce(0, effort);

          // Save the joint state
          currRobotState.name.push_back(jointName);
          currRobotState.position.push_back(name_it->second->GetAngle(0).Radian());
          currRobotState.velocity.push_back(name_it->second->GetVelocity(0));
          currRobotState.effort.push_back(effort);
        }

        // Publish the robot's current state if the appropriate
        // amount of time has passed

        if ((currRobotState.header.stamp - lastPubTime).toSec() > 1.0 / 1000)
        {
          this->pub.publish(currRobotState);
          lastPubTime = currRobotState.header.stamp;
        }
        mutex.unlock();
      }
    }

    std::string jointStateToString(const controlit_robot_interface_library::JointState::ConstPtr & msg)
    {
      std::stringstream buff;

      buff << "\n        Header:\n";
      buff << "          seq: " << msg->header.seq << "\n";
      buff << "          stamp: " << ros::Time(msg->header.stamp.sec, msg->header.stamp.nsec).toSec() << "\n";
      buff << "          frame_id: " << msg->header.frame_id << "\n";

      size_t numDOFs = msg->name.size();

      buff << "        Name: [";

      for (size_t ii = 0; ii < numDOFs; ii++)
      {
        buff << msg->name[ii];
        if (ii < numDOFs - 1)
          buff << ", ";
      }

      buff << "]\n";

      numDOFs = msg->position.size();

      buff << "        Position: [";

      for (size_t ii = 0; ii < numDOFs; ii++)
      {
        buff << msg->position[ii];
        if (ii < numDOFs - 1)
          buff << ", ";
      }

      buff << "]\n";

      numDOFs = msg->velocity.size();

      buff << "        Velocity: [";

      for (size_t ii = 0; ii < numDOFs; ii++)
      {
        buff << msg->velocity[ii];
        if (ii < numDOFs - 1)
          buff << ", ";
      }

      buff << "]\n";


      numDOFs = msg->effort.size();

      buff << "        Effort: [";

      for (size_t ii = 0; ii < numDOFs; ii++)
      {
        buff << msg->effort[ii];
        if (ii < numDOFs - 1)
          buff << ", ";
      }

      buff << "]";

      return buff.str();
    }

    void commandCallback(const controlit_robot_interface_library::JointState::ConstPtr & msg)
    {
      gazebo::common::Console::msg << "Received command:\n" << jointStateToString(msg) << "\n";

      // Save the rtt sequence number
      rttSeqNo = msg->rtt_seq_no;

      // Ensure the number of DOFs in the command message matches the expected number
      size_t numDOFs = msg->name.size();

      if (numDOFs == joints.size())
      {
        if (mutex.try_lock())
        {
          for (size_t ii = 0; ii < numDOFs; ii++)
          {
            std::string currJointName = msg->name[ii];
            double effort = msg->effort[ii];

            if (torques.find(currJointName) != torques.end())
            {
              torques[currJointName] = effort;
              // joints[currJointName]->SetForce(0, effort); // the zero is the axis index
            }
            else
              gzerr << "Unknown joint \"" << currJointName << "\", not saving effort command.\n";
          }
          mutex.unlock();
        }
      }
      else
        gzerr << "Incorrect command size, expected " << joints.size() << ", got " << numDOFs << ".\n";
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // ROS Nodehandle
    private: ros::NodeHandle* node;

    // ROS Subscriber
    ros::Subscriber sub;

    // ROS Publisher
    ros::Publisher pub;

  private:

    // Stores a pointer to each joint
    std::map<std::string, physics::JointPtr> joints;

    // Stores the torque command for each joint
    std::map<std::string, double> torques;

    // The time when the robot's state was last published
    ros::Time lastPubTime;

    std::mutex mutex;

    /*!
     * A sequence number for measuring round-trip communication latency between the
     * controller and the robot.
     */
    uint8_t rttSeqNo;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ControlItROSTopicPlugin)
}
