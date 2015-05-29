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
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
// #include "sensor_msgs/JointState.h"

#include "controlit_udp/TxRxUDP.hpp"
#include "controlit_udp/UDPDelayServer.hpp"

#include <XmlRpcValue.h>
#include <XmlRpcException.h>

// Robot sends/listens on these ports--WBC will use default.
#define ROBOT_STATE_PORT 51126
#define ROBOT_CMD_PORT 51127

namespace gazebo {

/*!
 * A plugin that connects Trikey in Gazebo with a controller over a UDP
 * connection.
 */
class TrikeyUDPEmbeddedControllerPlugin : public ModelPlugin
{
    public: TrikeyUDPEmbeddedControllerPlugin() :
    cmdDelay_ms(0), stateDelay_ms(0), servoRate_hz(1000)
    {
        // Start up ROS
        std::string name = "trikey_udp_plugin_with_embedded_control";
        int argc = 0;
        ros::init(argc, NULL, name);

        // Initialize variables
        lastPubTime = ros::Time::now();

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

    public: ~TrikeyUDPEmbeddedControllerPlugin()
    {
      delete this->node;
      delete[] this->JOINT_NAMES;
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        // Store the pointer to the model
        this->model = _parent;

        // Load parameters for this plugin
        for (size_t ii = 0; ii < 3; ii++)
        {
            physics::JointPtr _jptr = this->model->GetJoint(JOINT_NAMES[ii]);
            joints.insert(std::make_pair(JOINT_NAMES[ii], _jptr));
            kp.insert(std::make_pair(_jptr->GetName(), 0));
            kd.insert(std::make_pair(_jptr->GetName(), 0));
            goalPosition.insert(std::make_pair(_jptr->GetName(), 0));
            goalVelocity.insert(std::make_pair(_jptr->GetName(), 0));

            // Initialize the state message
            stateMsg.state.position[ii] = 0;
            stateMsg.state.velocity[ii] = 0;
            stateMsg.state.effort[ii] = 0;
        }

        // ROS Nodehandle
        this->node = new ros::NodeHandle;

        // Get delay and rate parameters from the server...TODO: quit hard-coding these!!
        if(this->node->hasParam("/trikey_controller/state_delay_ms"))
            this->node->getParam("/trikey_controller/state_delay_ms", stateDelay_ms);

        if(this->node->hasParam("/trikey_controller/command_delay_ms"))
            this->node->getParam("/trikey_controller/command_delay_ms", cmdDelay_ms);

        // this->node->getParam("/trikey_controller/controlit_update_rate",servoRate_hz);

        gzdbg << "cmdDelay_ms = " << cmdDelay_ms << ", stateDelay_ms = " << stateDelay_ms << ", servoRate_hz = " << servoRate_hz << "\n";

        // Create the UDP interface.  For now, hard-code the default values.
        std::cerr << "TrikeyUDPEmbeddedControllerPlugin::Load: Creating UDP interface with the following properties:\n"
                  << "  - address: " << DEFAULT_INET_ADDR << "\n"
                  << "  - statePort: " << ROBOT_STATE_PORT << "\n"
                  << "  - cmdPort: " << ROBOT_CMD_PORT << "\n";

        if (!UDP.init(&stateMsg, &cmdMsg, ROBOT_STATE_PORT, ROBOT_CMD_PORT, DEFAULT_INET_ADDR))
            gzerr<<"something failed in UDP socket inititalization \n";

        // Initialize the UDP delay server.
        std::cerr << "TrikeyUDPEmbeddedControllerPlugin::Load: Creating UDPDelay component with the following properties:\n"
                  << "  - address: " << DEFAULT_INET_ADDR << "\n"
                  << "  - rxStatePort: " << ROBOT_STATE_PORT << "\n"
                  << "  - rxCmdPort: " << DEFAULT_CMD_PORT << "\n"
                  << "  - txStatePort: " << DEFAULT_STATE_PORT << "\n"
                  << "  - txCmdPort: " << ROBOT_CMD_PORT << "\n"
                  << "  - stateDelay_ms: " << stateDelay_ms << "\n"
                  << "  - cmdDelay_ms: " << cmdDelay_ms << "\n"
                  << "  - servoRate_hz: " << servoRate_hz << "\n";

        if (!UDPDelay.init(ROBOT_STATE_PORT, DEFAULT_CMD_PORT, DEFAULT_STATE_PORT, ROBOT_CMD_PORT, DEFAULT_INET_ADDR, stateDelay_ms, cmdDelay_ms, servoRate_hz))
            gzerr<<"something failed in delay server initialization \n";

        // Listeners for published updates to embedded controller
        this->jointPositionGoalSub = this->node->subscribe("embedded_goal_position", 1000, &TrikeyUDPEmbeddedControllerPlugin::GoalJointPositionCallback, this);
        this->jointVelocityGoalSub = this->node->subscribe("embedded_goal_velocity", 1000, &TrikeyUDPEmbeddedControllerPlugin::GoalJointVelocityCallback, this);
        this->kpSub = this->node->subscribe("embedded_kp", 1000, &TrikeyUDPEmbeddedControllerPlugin::KpCallback, this);
        this->kdSub = this->node->subscribe("embedded_kd", 1000, &TrikeyUDPEmbeddedControllerPlugin::KdCallback, this);
        this->checkParametersSub = this->node->subscribe("check_delay_parameters", 1000, &TrikeyUDPEmbeddedControllerPlugin::checkDelayParametersCallback, this);

        // Listen to the update event. This event is broadcast every simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&TrikeyUDPEmbeddedControllerPlugin::onUpdate, this));

        // gzdbg << "Load: Done method call.\n";
    }

    public: void checkDelayParametersCallback(const std_msgs::Bool bool_msgs)
    {
        double newSDelay = stateDelay_ms;
        double newCDelay = cmdDelay_ms;
        bool gotStateDelay = false;
        bool gotCmdDelay = false;

        if(this->node->hasParam("/trikey_controller/state_delay_ms"))
            gotStateDelay = this->node->getParam("/trikey_controller/state_delay_ms", newSDelay);

        if(this->node->hasParam("/trikey_controller/command_delay_ms"))
            gotCmdDelay = this->node->getParam("/trikey_controller/command_delay_ms", newCDelay);

        if(gotStateDelay)
        {
            if(newSDelay != stateDelay_ms)
                stateDelay_ms = newSDelay;
            else
                gotStateDelay = false;
        }

        if(gotCmdDelay)
        {
            if(newCDelay != cmdDelay_ms)
                cmdDelay_ms = newCDelay;
            else
                gotCmdDelay = false;
        }

        if(gotStateDelay || gotCmdDelay)
        {
            if(gotStateDelay && gotCmdDelay)
            {
                UDPDelay.setDelay(stateDelay_ms, cmdDelay_ms);
            }
            else
            {
                if(gotStateDelay && !gotCmdDelay)
                {
                    UDPDelay.setStateDelay(stateDelay_ms);
                }
                else
                {
                    UDPDelay.setCommandDelay(cmdDelay_ms);
                }
            }
        }

        gzdbg<<"cmdDelay_ms = " << cmdDelay_ms << ", stateDelay_ms = " << stateDelay_ms << "\n";
    }

    public: void GoalJointPositionCallback(const std_msgs::Float64MultiArray goal_msg)
    {
        int ii = 0;
        std::map<std::string, physics::JointPtr>::iterator pJnt;
        for (pJnt = joints.begin(); pJnt != joints.end(); pJnt ++)
            goalPosition[pJnt->first] = goal_msg.data[ii++];

        gzdbg << "Embedded Goal Position callback\n";
    }

    public: void GoalJointVelocityCallback(const std_msgs::Float64MultiArray goal_msg)
    {
        int ii = 0;
        std::map<std::string, physics::JointPtr>::iterator pJnt;
        for (pJnt = joints.begin(); pJnt != joints.end(); pJnt ++)
            goalVelocity[pJnt->first] = goal_msg.data[ii++];

        gzdbg << "Embedded Goal Velocity callback\n";
    }

    public: void KpCallback(const std_msgs::Float64MultiArray gain_msg)
    {
        int ii = 0;
        std::map<std::string, physics::JointPtr>::iterator pJnt;
        for (pJnt = joints.begin(); pJnt != joints.end(); pJnt ++)
            kp[pJnt->first] = gain_msg.data[ii++];

        gzdbg << "Embedded Kd callback\n";
    }

    public: void KdCallback(const std_msgs::Float64MultiArray gain_msg)
    {
        int ii = 0;
        std::map<std::string, physics::JointPtr>::iterator pJnt;
        for (pJnt = joints.begin(); pJnt != joints.end(); pJnt ++)
            kd[pJnt->first] = gain_msg.data[ii++];

        gzdbg << "Embedded Kd callback\n";
    }

    // Called by the world update start event
    public: void onUpdate()
    {
        // Process pending ROS messages, etc.
        ros::spinOnce();

        // Process command message
        // gzdbg << "Received command:\n  - effort: "
        //     << cmdMsg.effort[0] << ", " << cmdMsg.effort[1] << ", " << cmdMsg.effort[2] << "\n";

        for (size_t ii = 0; ii < 3; ii++)
        {
            physics::JointPtr currJoint = joints[JOINT_NAMES[ii]];
            double th = currJoint->GetAngle(0).Radian();
            double th_dot = currJoint->GetVelocity(0);

            double tau = cmdMsg.command.effort[ii] + kp[JOINT_NAMES[ii]] * (goalPosition[JOINT_NAMES[ii]] - th) + kd[JOINT_NAMES[ii]] * (goalVelocity[JOINT_NAMES[ii]] - th_dot);

            currJoint->SetForce(0, tau); // the zero is the axis index
        }

        // Process state message

        for (int ii = 0; ii < 3; ii++)
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

    // Delay parameters
    double cmdDelay_ms;
    double stateDelay_ms;
    double servoRate_hz;

    // intercepts two-way communication and introcuces delay
    controlit_udp::UDPDelayServer UDPDelay;

    // controlit comm udp implements socket stuff
    controlit_udp::TxStateRxCommandUDP UDP;

    // The message to send over UDP.
    controlit_udp::StateMsg stateMsg;

    // The most recently received command.
    controlit_udp::CommandMsg cmdMsg;

    // The time when the robot's state was last published
    ros::Time lastPubTime;

    // Subscribers to ros topics for embedded controls
    ros::Subscriber jointPositionGoalSub;
    ros::Subscriber jointVelocityGoalSub;
    ros::Subscriber kpSub;
    ros::Subscriber kdSub;

    // Subscribers for UPD delay server
    ros::Subscriber checkParametersSub;

    // Embedded control parameters
    std::map<std::string, double> kp;
    std::map<std::string, double> kd;
    std::map<std::string, double> goalPosition;
    std::map<std::string, double> goalVelocity;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(TrikeyUDPEmbeddedControllerPlugin)

} // namespace gazebo
