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
// #include "std_msgs/Header.h"
// #include "std_msgs/Float64.h"
// #include "std_msgs/Float64MultiArray.h"
// #include "sensor_msgs/JointState.h"

#include "controlit_udp/TxRxUDP.hpp"

#include <XmlRpcValue.h>
#include <XmlRpcException.h>

#define JOINT_STATE_PUBLISH_FREQ_HZ 400

namespace gazebo {
namespace common {
    #define gzout (gazebo::common::Console::Instance()->ColorErr("Dbg",__FILE__, __LINE__,36))
    #define gzprint (gazebo::common::Console::Instance()->ColorErr("FYI:",__FILE__, __LINE__,25))
}  // namespace common
}  // namespace gazebo

namespace gazebo {

/*!
 * A plugin that connects Trikey in Gazebo with a controller over a UDP
 * connection.
 */
class TrikeyUDPPlugin : public ModelPlugin
{
    public: TrikeyUDPPlugin()
    {
        // Start up ROS
        std::string name = "trikey_udp_plugin";
        int argc = 0;
        ros::init(argc, NULL, name);

        // Initialize variables
        lastPubTime = ros::Time::now();

        // TODO : GET THIS FROM PARAMETER!!!!!
        JOINT_NAMES = new std::string[3]
        {
          "base_to_wheel_j0",
          "base_to_wheel_j1",
          "base_to_wheel_j2"
        };

        stateMsg.seqno = -1;
        cmdMsg.seqno = -1;
        stateMsg.state.num_dofs = 3;
        cmdMsg.command.num_dofs = 3;
    }

    public: ~TrikeyUDPPlugin()
    {
      delete this->node;
      delete[] this->JOINT_NAMES;
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        // Store the pointer to the model
        this->model = _parent;

        //TODO : FIX THIS!!!!! COUNT MOVABLE JOINTS!!!
        // Load parameters for this plugin
        for (size_t ii = 0; ii < 3; ii++)
        {
            physics::JointPtr _jptr = this->model->GetJoint(JOINT_NAMES[ii]);
            joints.insert(std::make_pair(JOINT_NAMES[ii], _jptr));
            // torques.insert(std::make_pair(JOINT_NAMES[ii], 0.0));

            // Initialize the state message
            stateMsg.state.position[ii] = 0;
            stateMsg.state.velocity[ii] = 0;
            stateMsg.state.effort[ii] = 0;
        }

        // Create the UDP interface.  For now, use the default values.  Starts receive thread.
        std::cerr << "TrikeyUDPPlugin::Load: Creating UDP interface with the following properties:\n"
                  << "  - address: "   << DEFAULT_INET_ADDR  << "\n"
                  << "  - statePort: " << DEFAULT_STATE_PORT << "\n"
                  << "  - cmdPort: "   << DEFAULT_CMD_PORT   << "\n";

        if (!UDP.init(&stateMsg, &cmdMsg))
            gzerr<<"something failed in UDP socket inititalization";

        // ROS Nodehandle
        this->node = new ros::NodeHandle;

        // Listen to the update event. This event is broadcast every simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&TrikeyUDPPlugin::onUpdate, this));

    }

    // Called by the world update start event
    public: void onUpdate()
    {
        // Process pending ROS messages, etc.
        ros::spinOnce();

        // Process received command

        // gzprint << "Received command:\n  - effort: "
        //     << cmdMsg.effort[0] << ", " << cmdMsg.effort[1] << ", " << cmdMsg.effort[2] << "\n";

        for (size_t ii = 0; ii < 3; ii++)
        {
            physics::JointPtr currJoint = joints[JOINT_NAMES[ii]];
            currJoint->SetForce(0, cmdMsg.command.effort[ii]);       // the first parameter is the axis index
        }

        
        // Send state message
        for (size_t ii = 0; ii < 3; ii++)
        {
            physics::JointPtr currJoint = joints[JOINT_NAMES[ii]];

            stateMsg.state.position[ii] = currJoint->GetAngle(0).Radian();
            stateMsg.state.velocity[ii] = currJoint->GetVelocity(0);
            stateMsg.state.effort[ii] = currJoint->GetForce(0);
        }

        // gzprint << "Reflecting sequence number " << cmdMsg.seqno << "\n";
        stateMsg.seqno = cmdMsg.seqno;

        UDP.sendState();
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // ROS Nodehandle
    private: ros::NodeHandle * node;

private:

    // The joint names (order should match order in message).
    std::string * JOINT_NAMES;

    // Stores a pointer to each joint
    std::map<std::string, physics::JointPtr> joints;

    // ControlIt! comm udp implements socket stuff
    controlit_udp::TxStateRxCommandUDP UDP;

    // The message to send over UDP.
    controlit_udp::StateMsg stateMsg;

    // The most recently received command.
    controlit_udp::CommandMsg cmdMsg;

    // The time when the robot's state was last published
    ros::Time lastPubTime;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(TrikeyUDPPlugin)

} // namespace gazebo
