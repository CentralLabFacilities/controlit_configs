#include <ros/ros.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <stdio.h>
#include <unistd.h>
#include <mutex>
#include <thread>
#include <chrono>

#include <std_msgs/Int64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>

#include <shared_memory_interface/shared_memory_publisher.hpp>
#include <shared_memory_interface/shared_memory_subscriber.hpp>


namespace gazebo {

// #define LOCKSTEP 0
// #define LOCKSTEP_TIMEOUT 0.05 //seconds
#define PRINT_STATE_SENT 0
#define PRINT_COMMAND_RECEIVED 0

#define ADD_STICTION 0
#define STICTION_THRESHOLD_VELOCITY 0.1
#define STICTION_THRESHOLD_FORCE 10

// Parameters for shared memory subscribers
#define LISTEN_TO_ROS_TOPIC false
#define USE_POLLING false

class SMControlPlugin: public ModelPlugin
{
public:
    SMControlPlugin() :
        cmdSubscriber(LISTEN_TO_ROS_TOPIC, USE_POLLING),
        rttTxSubscriber(LISTEN_TO_ROS_TOPIC, USE_POLLING),
        rcvdCmd(false),
        rcvdRTT(false),
        isFirstSend(true),
        rcvdInitCmdMsg(false),
        rcvdInitRTTMsg(false)
        // idleThreadOn(false)
    {
        noCmdErrLastPrintTime = ros::Time::now(); // initialization
    }

    ~SMControlPlugin()
    {
    }

    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        // gzmsg << __func__ << ": Method called!\n";
  
        // Initialize ROS if necessary
        if (!ros::isInitialized())
        {
            int argc = 0;
            char** argv = NULL;
            ros::init(argc, argv, "SMControlPlugin", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
        }
  
        // Store a pointer to the model
        this->model = _parent;
  
        // Listen to the update event. This event is broadcasted every simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&SMControlPlugin::OnUpdate, this, _1));
  
        // Create a ROS node handle.  This is used to access the ROS parameter server.
        this->rosNode = new ros::NodeHandle("");
  
        // By default, expose every joint in the robot.  See below where we
        // adjust this number based on a user-supplied joint order list.
        unsigned long num_joints = this->model->GetJoints().size();
  
        // Get Gazebo's list of joints
        const gazebo::physics::Joint_V jointListGazebo = this->model->GetJoints();
  
        // Allocate a vector for holding the joint order in shared memory
        std::vector<std::string> jointOrderNameInSM;
  
        // Check if the user specified an alternative number of DOFs and joint order.  
        ROS_INFO_STREAM("SMControlPlugin-" << getpid() << "::" << __func__ << ": Checking ROS parameter " << this->rosNode->getNamespace() << "/ControlItSMGazeboPlugin/JointOrder for joint order...");
        XmlRpc::XmlRpcValue jointOrderList;
        if(this->rosNode->getParam("ControlItSMGazeboPlugin/JointOrder", jointOrderList))
        {
            ROS_ASSERT(jointOrderList.getType() == XmlRpc::XmlRpcValue::TypeArray);
    
            num_joints = jointOrderList.size();
            ROS_INFO_STREAM("SMControlPlugin-" << getpid() << "::" << __func__ << ": Using alternate number of DOFs: " << num_joints);
            
            // Store the joint order in jointOrderNameInSM, which is a vector of strings
            for(int32_t ii = 0; ii < jointOrderList.size(); ++ii)
            {
                ROS_ASSERT(jointOrderList[ii].getType() == XmlRpc::XmlRpcValue::TypeString);
    
                // ROS_INFO_STREAM("Index " << ii << ", joint name: " << static_cast<std::string>(jointOrderList[ii]));
                jointOrderNameInSM.push_back(static_cast<std::string>(jointOrderList[ii]));
            }
    
            // Initialize the joint_index_sm_to_gazebo_map.  This vector is indexed by shared memory location
            // and the value is the the Gazebo index.
            for(size_t ii = 0; ii < jointOrderNameInSM.size(); ii++)
            {
                bool jointFound = false;
                for(size_t jj = 0; !jointFound && jj < jointListGazebo.size(); jj++)
                {
                    if(jointListGazebo[jj]->GetName().compare(jointOrderNameInSM[ii]) == 0)
                    {
                        std::cerr << "SMControlPlugin-" << getpid() << "::" << __func__ << ": Shared memory joint " << ii << " (" << jointOrderNameInSM[ii] << ") is Gazebo joint index " << jj << std::endl;
          
                        joint_index_sm_to_gazebo_map.push_back(jj);
                        jointFound = true;
                    }
                }
      
                if(!jointFound)
                {
                    ROS_ERROR_STREAM("SMControlPlugin-" << getpid() << "::" << __func__ << ": Unable to get Gazebo index for joint " << ii << " named " << jointOrderNameInSM[ii]);
                    ros::shutdown();
                    return;
                }
            }
        }
        else
        {
            // The user did not specify a custom shared memory joint order.  Make the SM index the same as the Gazebo index.
            for(size_t ii = 0; ii < this->model->GetJoints().size(); ii++)
            {
                joint_index_sm_to_gazebo_map.push_back(ii);
                jointOrderNameInSM.push_back(jointListGazebo[ii]->GetName());
            }
        }
  
        // Print the joint order (useful for debugging purposes)
        {
            std::stringstream ss;
            for (size_t ii = 0; ii < joint_index_sm_to_gazebo_map.size(); ii++)
            {
                size_t gazeboIndex = joint_index_sm_to_gazebo_map[ii];
                ss << "  " << ii << ": " << jointListGazebo[gazeboIndex]->GetName() << "\n";
            }
            std::cerr << "SMControlPlugin-" << getpid() << "::" << __func__ << ": jointOrderNameInSM:\n" << ss.str() << std::endl;
        }
  
        // Check if a list of actuated joint is provided.  If it is, load it.
        XmlRpc::XmlRpcValue unactuatedJointList;
        if(this->rosNode->getParam("ControlItSMGazeboPlugin/UnactuatedJoints", unactuatedJointList))
        {
            ROS_ASSERT(unactuatedJointList.getType() == XmlRpc::XmlRpcValue::TypeArray);
    
            // For each joint in Gazebo, if it appears in the unactuated joint list,
            // save a 1 in its position in the unactuated_joint_mask.
            for(size_t jj = 0; jj < jointListGazebo.size(); jj++)
            {
                bool jointIsMasked = false;
      
                for(int kk = 0; !jointIsMasked && kk < unactuatedJointList.size(); kk++)
                {
                    ROS_ASSERT(unactuatedJointList[kk].getType() == XmlRpc::XmlRpcValue::TypeString);
        
                    if(jointListGazebo[jj]->GetName().compare(unactuatedJointList[kk]) == 0)
                    {
                        // std::cerr << "SMControlPlugin::Load: Joint \"" << unactuatedJointList[kk] << "\" (Gazebo index " << jj << ") is unactuated!" << std::endl;
                        jointIsMasked = true;
                    }
                }
      
                unactuated_joint_mask.push_back(jointIsMasked? 1 : 0);
            }
        }
        else
        {
            // No unactuated joints.  Initialize unactuated_joint_mask to contain all zeros.
            for(size_t jj = 0; jj < jointListGazebo.size(); jj++)
            {
                unactuated_joint_mask.push_back(0);
            }
        }
  
        // Print the unactuated joint mask
        {
            std::stringstream ss;
            for (size_t ii = 0; ii < unactuated_joint_mask.size(); ii++)
            {
                ss << "  - Joint \"" << jointListGazebo[ii]->GetName() << "\" " << (unactuated_joint_mask[ii] == 1 ? "is NOT actuable" : "is actuable") << std::endl;  
            }
            std::cerr << "SMControlPlugin-" << getpid() << "::" << __func__ << ": unactuated_joint_mask:" << std::endl << ss.str();
        }
        
        // Declare the shared memory I/O ports
        std::cerr << "SMControlPlugin-" << getpid() << "::" << __func__ << ": Setting up /joint_states publisher" << std::endl;
        robotStatePublisher.advertise("/joint_states");
  
        std::cerr << "SMControlPlugin-" << getpid() << "::" << __func__ << ": Setting up /rtt_rx publisher" << std::endl;
        rttRxPublisher.advertise("/rtt_rx");
  
        std::cerr << "SMControlPlugin-" << getpid() << "::" << __func__ << ": Setting up /cmd subscriber" << std::endl;
        cmdSubscriber.subscribe("/cmd", boost::bind(&SMControlPlugin::commandCallback, this, _1));

        std::cerr << "SMControlPlugin-" << getpid() << "::" << __func__ << ": Setting up /rtt_tx subscriber" << std::endl;
        rttTxSubscriber.subscribe("/rtt_tx", boost::bind(&SMControlPlugin::rttCallback, this, _1));
  
        // Initialize the rtt_rx value to be zero and publish an initial message
        rttMsg.data = 0;
        rttRxPublisher.publish(rttMsg);
  
        // Obtain the odometry topic from the ROS parameter server
        // std::cerr << "SMControlPlugin::Load: Accessing odometry topic from ROS parameter: " << this->rosNode->getNamespace() << "/odometry_topic" << std::endl;
        if(this->rosNode->getParam("controlit/odometry_topic", rootLinkOdomTopic))
        {
            std::cerr << "SMControlPlugin-" << getpid() << "::" << __func__ << ": Publishing to odometry topic " << rootLinkOdomTopic << std::endl;
        }
        else
        {
            std::cerr << "SMControlPlugin-" << getpid() << "::" << __func__ << ": No odometry topic specified on ROS parameter " 
                      << this->rosNode->getNamespace() << "/controlit/odometry_topic.  "
                      << "Defaulting to ROS topic /gazebo/root_link_odom" << std::endl;
            rootLinkOdomTopic = "/gazebo/root_link_odom";
        }      
        odometryPublisher.advertise(rootLinkOdomTopic);
  
        // Initialize the odometry state in the shared memory
        sendOdometry();
  
        // Initialize the joint state message and write it to shared memory
        initJointStateMsg();
        sendJointState();

        // // Wait for messages to arrive to ensure connection 
        // // is initialized prior to starting the controller.
        // bool shInitialized = false, rcvdCmdMsg = false, rcvdRTTMsg = false;

        // while (!shInitialized)
        // {
        //     std::cerr << "SMControlPlugin (" << getpid() << "), " << __func__ << ": Waiting for messages..." << std::endl;

        //     if (!cmdSubscriber.waitForMessage(cmdMsg, 1))
        //     {
        //         std::cerr << "SMControlPlugin (" << getpid() << "), " << __func__ << ": Initial command message not received..." << std::endl;
        //     }
        //     else
        //     {
        //         rcvdCmdMsg = true;
        //         std::cerr << "SMControlPlugin (" << getpid() << "), " << __func__ << ": Initial command message received." << std::endl;
        //     }

        //     if (!rttTxSubscriber.waitForMessage(rttMsg, 1))
        //     {
        //         std::cerr << "SMControlPlugin (" << getpid() << "), " << __func__ << ": Initial RTT TX message not received..." << std::endl;
        //     }
        //     else
        //     {
        //         rcvdRTTMsg = true;
        //         std::cerr << "SMControlPlugin (" << getpid() << "), " << __func__ << ": Initial RTT TX message received." << std::endl;
        //     }

        //     shInitialized = rcvdCmdMsg && rcvdRTTMsg;

        //     if (!shInitialized)
        //     {
        //         std::this_thread::sleep_for(std::chrono::milliseconds(100));
        //     }
        // }
        
        // thread = std::thread(&SMControlPlugin::idleLoop, this);
        std::cerr << "SMControlPlugin-" << getpid() << "::" << __func__ << ": Done loading plugin." << std::endl;
    }

    /*!
     * Obtains the latest root link odometry state and save it into shared memory.
     */
    void sendOdometry()
    {
        // Root link odometry
        gazebo::physics::Link_V links = this->model->GetLinks();
  
        gazebo::math::Pose pelvis_pose = links.at(0)->GetWorldPose();
        gazebo::math::Vector3 pelvis_linear_vel = links.at(0)->GetWorldLinearVel();
        gazebo::math::Vector3 pelvis_angular_vel = links.at(0)->GetWorldAngularVel();
        
        odomMsg.pose.pose.position.x = pelvis_pose.pos.x;
        odomMsg.pose.pose.position.y = pelvis_pose.pos.y;
        odomMsg.pose.pose.position.z = pelvis_pose.pos.z;
  
        // Get the orientation
        odomMsg.pose.pose.orientation.w = pelvis_pose.rot.w;
        odomMsg.pose.pose.orientation.x = pelvis_pose.rot.x;
        odomMsg.pose.pose.orientation.y = pelvis_pose.rot.y;
        odomMsg.pose.pose.orientation.z = pelvis_pose.rot.z;
  
        // Get the velocity
        odomMsg.twist.twist.linear.x = pelvis_linear_vel.x;
        odomMsg.twist.twist.linear.y = pelvis_linear_vel.y;
        odomMsg.twist.twist.linear.z = pelvis_linear_vel.z;
        odomMsg.twist.twist.angular.x = pelvis_angular_vel.x;
        odomMsg.twist.twist.angular.y = pelvis_angular_vel.y;
        odomMsg.twist.twist.angular.z = pelvis_angular_vel.z;
  
        // std::stringstream ss;
  
        // ss << "  odomMsg.pose.pose.position.x = " << pelvis_pose.pos.x << std::endl;
        // ss << "  odomMsg.pose.pose.position.y = " << pelvis_pose.pos.y << std::endl;
        // ss << "  odomMsg.pose.pose.position.z = " << pelvis_pose.pos.z << std::endl;
  
        // // Get the orientation
        // ss << "  odomMsg.pose.pose.orientation.w = " << pelvis_pose.rot.w << std::endl;
        // ss << "  odomMsg.pose.pose.orientation.x = " << pelvis_pose.rot.x << std::endl;
        // ss << "  odomMsg.pose.pose.orientation.y = " << pelvis_pose.rot.y << std::endl;
        // ss << "  odomMsg.pose.pose.orientation.z = " << pelvis_pose.rot.z << std::endl;
  
        // // Get the velocity
        // ss << "  odomMsg.twist.twist.linear.x = " << pelvis_linear_vel.x << std::endl;
        // ss << "  odomMsg.twist.twist.linear.y = " << pelvis_linear_vel.y << std::endl;
        // ss << "  odomMsg.twist.twist.linear.z = " << pelvis_linear_vel.z << std::endl;
        // ss << "  odomMsg.twist.twist.angular.x = " << pelvis_angular_vel.x << std::endl;
        // ss << "  odomMsg.twist.twist.angular.y = " << pelvis_angular_vel.y << std::endl;
        // ss << "  odomMsg.twist.twist.angular.z = " << pelvis_angular_vel.z << std::endl;
  
        // std::cerr << "Publishing odometry message: " << std::endl << ss.str();
  
        // odomMsg.pose.pose.position.x = -0.0236367;
        // odomMsg.pose.pose.position.y = -1.5868e-06;
        // odomMsg.pose.pose.position.z = 0.549475;
        // odomMsg.pose.pose.orientation.w = 0.984304;
        // odomMsg.pose.pose.orientation.x = -7.35029e-08;
        // odomMsg.pose.pose.orientation.y = 0.176484;
        // odomMsg.pose.pose.orientation.z = 2.31056e-06;
        // odomMsg.twist.twist.linear.x = -0.00103856;
        // odomMsg.twist.twist.linear.y = -1.27799e-05;
        // odomMsg.twist.twist.linear.z = 0.000200122;
        // odomMsg.twist.twist.angular.x = 4.3889e-06;
        // odomMsg.twist.twist.angular.y = -0.00147495;
        // odomMsg.twist.twist.angular.z = 3.6186e-05;
  
        {
            // Check for legit odometry values
            bool isLegit = true;
            if(std::abs(odomMsg.pose.pose.position.x) >= 1e3) {ROS_WARN_STREAM("SMControlPlugin (" << getpid() << "): " << __func__ << ": Odometry contained questionable value for odomMsg.pose.pose.position.x: " << odomMsg.pose.pose.position.x); isLegit = false; }
            if(std::abs(odomMsg.pose.pose.position.y) >= 1e3) {ROS_WARN_STREAM("SMControlPlugin (" << getpid() << "): " << __func__ << ": Odometry contained questionable value for odomMsg.pose.pose.position.y: " << odomMsg.pose.pose.position.y); isLegit = false; }
            if(std::abs(odomMsg.pose.pose.position.z) >= 1e3) {ROS_WARN_STREAM("SMControlPlugin (" << getpid() << "): " << __func__ << ": Odometry contained questionable value for odomMsg.pose.pose.position.z: " << odomMsg.pose.pose.position.z); isLegit = false; }
            if(std::abs(odomMsg.pose.pose.orientation.w) >= 1e3) {ROS_WARN_STREAM("SMControlPlugin (" << getpid() << "): " << __func__ << ": Odometry contained questionable value for odomMsg.pose.pose.orientation.w: " << odomMsg.pose.pose.orientation.w); isLegit = false; }
            if(std::abs(odomMsg.pose.pose.orientation.x) >= 1e3) {ROS_WARN_STREAM("SMControlPlugin (" << getpid() << "): " << __func__ << ": Odometry contained questionable value for odomMsg.pose.pose.orientation.x: " << odomMsg.pose.pose.orientation.x); isLegit = false; }
            if(std::abs(odomMsg.pose.pose.orientation.y) >= 1e3) {ROS_WARN_STREAM("SMControlPlugin (" << getpid() << "): " << __func__ << ": Odometry contained questionable value for odomMsg.pose.pose.orientation.y: " << odomMsg.pose.pose.orientation.y); isLegit = false; }
            if(std::abs(odomMsg.pose.pose.orientation.z) >= 1e3) {ROS_WARN_STREAM("SMControlPlugin (" << getpid() << "): " << __func__ << ": Odometry contained questionable value for odomMsg.pose.pose.orientation.z: " << odomMsg.pose.pose.orientation.z); isLegit = false; }
            if(std::abs(odomMsg.twist.twist.linear.x) >= 1e3) {ROS_WARN_STREAM("SMControlPlugin (" << getpid() << "): " << __func__ << ": Odometry contained questionable value for odomMsg.twist.twist.linear.x: " << odomMsg.twist.twist.linear.x); isLegit = false; }
            if(std::abs(odomMsg.twist.twist.linear.y) >= 1e3) {ROS_WARN_STREAM("SMControlPlugin (" << getpid() << "): " << __func__ << ": Odometry contained questionable value for odomMsg.twist.twist.linear.y: " << odomMsg.twist.twist.linear.y); isLegit = false; }
            if(std::abs(odomMsg.twist.twist.linear.z) >= 1e3) {ROS_WARN_STREAM("SMControlPlugin (" << getpid() << "): " << __func__ << ": Odometry contained questionable value for odomMsg.twist.twist.linear.z: " << odomMsg.twist.twist.linear.z); isLegit = false; }
            if(std::abs(odomMsg.twist.twist.angular.x) >= 1e3) {ROS_WARN_STREAM("SMControlPlugin (" << getpid() << "): " << __func__ << ": Odometry contained questionable value for: odomMsg.twist.twist.angular.x: " << odomMsg.twist.twist.angular.x); isLegit = false; }
            if(std::abs(odomMsg.twist.twist.angular.y) >= 1e3) {ROS_WARN_STREAM("SMControlPlugin (" << getpid() << "): " << __func__ << ": Odometry contained questionable value for: odomMsg.twist.twist.angular.y: " << odomMsg.twist.twist.angular.y); isLegit = false; }
            if(std::abs(odomMsg.twist.twist.angular.z) >= 1e3) {ROS_WARN_STREAM("SMControlPlugin (" << getpid() << "): " << __func__ << ": Odometry contained questionable value for: odomMsg.twist.twist.angular.z: " << odomMsg.twist.twist.angular.z); isLegit = false; }
    
            if (!isLegit) { assert(false); }
        }
  
        {
            #if PRINT_STATE_SENT
            std::cerr << "SMControlPlugin-" << getpid() << "::" << __func__ << ": Publishing odometry message..." << std::endl;
            #endif
        }

        // Publish the odometry information
        // std::cerr << "SMControlPlugin-" << getpid() << "::" << __func__ << ": Sending odometry..." << std::endl;
        odometryPublisher.publish(odomMsg);
        // std::cerr << "SMControlPlugin-" << getpid() << "::" << __func__ << ": Done sending odometry." << std::endl;
    }

    /*!
     * Initializes the joint state message.  Resizes the vectors to be of the correct length
     * with zero values.
     */
    void initJointStateMsg()
    {
        jointStateMsg.header.stamp = ros::Time::now();
        jointStateMsg.header.frame_id = "world";
  
        jointStateMsg.name.clear();
        jointStateMsg.position.clear();
        jointStateMsg.velocity.clear();
        jointStateMsg.effort.clear();
  
        const gazebo::physics::Joint_V jointListGazebo = this->model->GetJoints();
  
        for (size_t ii = 0; ii < joint_index_sm_to_gazebo_map.size(); ii++)
        {
            size_t gazeboIndex = joint_index_sm_to_gazebo_map[ii];
            jointStateMsg.name.push_back(jointListGazebo[gazeboIndex]->GetName());
            jointStateMsg.position.push_back(0);
            jointStateMsg.velocity.push_back(0);
            jointStateMsg.effort.push_back(0);
        }
    }

    /*!
     * Obtains the latest joint state save it into shared memory.
     */
    void sendJointState()
    {
        gazebo::physics::Joint_V joints = this->model->GetJoints();
  
        // std::vector<double> positions;
        // std::vector<double> velocities;
        // std::vector<double> torques;
  
        // Get the current joint states
        for(unsigned int ii = 0; ii < joint_index_sm_to_gazebo_map.size(); ii++)
        {
            size_t gazeboIndex = joint_index_sm_to_gazebo_map[ii];
    
            gazebo::physics::JointWrench gazebo_wrench = joints[gazeboIndex]->GetForceTorque(0);
            math::Vector3 axis = joints[gazeboIndex]->GetLocalAxis(0);
            double position = joints[gazeboIndex]->GetAngle(0).Radian();
            double velocity = joints[gazeboIndex]->GetVelocity(0);
            double torque = axis.x * gazebo_wrench.body2Torque.x + axis.y * gazebo_wrench.body2Torque.y + axis.z * gazebo_wrench.body2Torque.z;
    
            if(std::isnan(position) || std::isnan(position))
            {
                ROS_WARN_STREAM("SMControlPlugin-" << getpid() << "::" << __func__ << ": NAN or INF position found");
                position = 0;
            }
            if(std::isnan(velocity) || std::isnan(velocity))
            {
                ROS_WARN_STREAM("SMControlPlugin-" << getpid() << "::" << __func__ << ": NAN or INF velocity found");
                velocity = 0;
            }
            if(std::isnan(torque) || std::isnan(torque))
            {
                ROS_WARN_STREAM("SMControlPlugin-" << getpid() << "::" << __func__ << ": NAN or INF torque found");
                torque = 0;
            }
    
            {
                // check for legit values
                if (std::abs(position) >= 1e3) ROS_WARN_STREAM("SMControlPlugin (" << getpid() << "): " << __func__ << ": questionable position value for joint " << jointStateMsg.name[ii] << ": " << position);
                if (std::abs(velocity) >= 1e3) ROS_WARN_STREAM("SMControlPlugin (" << getpid() << "): " << __func__ << ": questionable velocity value for joint " << jointStateMsg.name[ii] << ": " << velocity);
                if (std::abs(torque) >= 1e3) ROS_WARN_STREAM("SMControlPlugin (" << getpid() << "): " << __func__ << ": questionable torque value for joint " << jointStateMsg.name[ii] << ": " << torque);
            }
    
            // positions.push_back(position);
            // velocities.push_back(velocity);
            // torques.push_back(torque);
            jointStateMsg.position[ii] = position;
            jointStateMsg.velocity[ii] = velocity;
            jointStateMsg.effort[ii] = torque;
        }
  
        // Print the state sent (useful for debugging purposes)
        {
            #if PRINT_STATE_SENT
            std::stringstream ss;
            ss << "SMControlPlugin-" << getpid() << "::" << __func__ << ": Sending:\n"
    
            << " - positions: [";
            for (size_t ii = 0; ii < joints.size(); ii++)
            {
                ss << jointStateMsg.position[ii]; if (ii < joints.size() - 1) ss << ", ";
            }
            ss << "]\n"
    
            << " - velocities: [";
            for (size_t ii = 0; ii < joints.size(); ii++)
            {
                ss << jointStateMsg.velocity[ii]; if (ii < joints.size() - 1) ss << ", ";
            }
            ss << "]\n"
            
            << " - efforts: [";
            for (size_t ii = 0; ii < joints.size(); ii++)
            {
                ss << jointStateMsg.effort[ii]; if (ii < joints.size() - 1) ss << ", ";
            }
            ss << "]";
    
            std::cerr << ss.str() << std::endl;
            #endif
        }
  
        // Write the joint state to shared memory
        // std::cerr << "SMControlPlugin-" << getpid() << "::" << __func__ << ": Sending joint state..." << std::endl;
        robotStatePublisher.publish(jointStateMsg);
        // std::cerr << "SMControlPlugin-" << getpid() << "::" << __func__ << ": Done sending joint state." << std::endl;
    }

    void rttCallback(std_msgs::Int64 & msg)
    {
        if (!rcvdInitRTTMsg)
        {
            std::cerr << "SMControlPlugin-" << getpid() << "::" << __func__ << ": received initial RTT message" << std::endl;
            rcvdInitRTTMsg = true;
        }
        else
        {
            rttMutex.lock();

            rttMsg = msg;
            rcvdRTT = true;

            rttMutex.unlock();
        }

    }

    void commandCallback(std_msgs::Float64MultiArray & msg)
    {
        if (!rcvdInitCmdMsg)
        {
            std::cerr << "SMControlPlugin-" << getpid() << "::" << __func__ << ": received initial cmd message" << std::endl;
            rcvdInitCmdMsg = true;
        }
        else
        {
            cmdMutex.lock();

            cmdMsg = msg;
            rcvdCmd = true;
        
            cmdMutex.unlock();
        }
    }

    /*!
     * If a command was received, send the torques to the Gazebo joints.
     */
    void receiveCmd()
    {
        cmdMutex.lock();
  
        // If a command was received, transmit it to Gazebo
        if (rcvdCmd)
        {
            gazebo::physics::Joint_V joints = this->model->GetJoints();
    
            // For each joint in shared memory
            for(unsigned int ii = 0; ii < joint_index_sm_to_gazebo_map.size(); ii++)
            {
                size_t gazeboIndex = joint_index_sm_to_gazebo_map[ii];
      
                // Only issue commands to actuated joints
                if(unactuated_joint_mask[gazeboIndex] == 0)
                {
                    if(ADD_STICTION && (std::abs(joints[gazeboIndex]->GetVelocity(0)) < STICTION_THRESHOLD_VELOCITY))
                    {
                        if(fabs(cmdMsg.data[ii]) > STICTION_THRESHOLD_FORCE)
                        {
                            joints[gazeboIndex]->SetForce(0, cmdMsg.data[ii]);
                        }
                        else
                        {
                            joints[gazeboIndex]->SetVelocity(0, 0.0);
                            joints[gazeboIndex]->SetForce(0, 0.0);
                        }
                    }
                    else
                    {
                        joints[gazeboIndex]->SetForce(0, cmdMsg.data[ii]); // model stiction
                    }
                }
            }
    
            // Check for legit command
            {
                bool isLegit = true;
                for(unsigned int ii = 0; ii < joint_index_sm_to_gazebo_map.size() && !isLegit; ii++)
                {
                    if (cmdMsg.data[ii] >= 1e3) isLegit = false;
                }    
                if (!isLegit)
                    std::cerr << "SMControlPlugin-" << getpid() << "::" << __func__ << ": Questionable values in command:" << std::endl << printCmd();
            }
    
            // Print command received (useful for debugging purposes)
            #if PRINT_COMMAND_RECEIVED
            std::cerr << "SMControlPlugin-" << getpid() << "::" << __func__ << ": Read command:" << std::endl << printCmd();
            #endif
          
        }
        else
        {
            // Only print this message if the simulation has been running for over 1 second.
            // Do not print it more than once very second.
            if (ros::Time::now().toSec() > 1)
            {

                if ((ros::Time::now() - noCmdErrLastPrintTime).toSec() > 1.0)
                {
                    std::cerr << "SMControlPlugin-" << getpid() << "::" << __func__ << ": Did not get command." << std::endl;
                    noCmdErrLastPrintTime = ros::Time::now();
                }
            }
        }
  
        cmdMutex.unlock();
    }

    /*!
     * Prints the command.  Note that this should only be called when the cmdMutex lock is held.
     */
    std::string printCmd()
    {
        gazebo::physics::Joint_V joints = this->model->GetJoints(); 
        std::stringstream ssRead;
        for(unsigned int ii = 0; ii < joint_index_sm_to_gazebo_map.size(); ii++)
        {
            size_t gazeboIndex = joint_index_sm_to_gazebo_map[ii];
            ssRead << "  Joint: " << joints[gazeboIndex]->GetName() << ", effort cmd: " << cmdMsg.data[ii] << std::endl;
        }
        return ssRead.str();
    }

    /*!
     * Reflects the RTT sequence number so the round-trip time can be computed by the controller.
     */
    void reflectRTT()
    {
        rttMutex.lock();
        if (rcvdRTT)
        {
            // std::cerr << "SMControlPlugin: reflectRTT: Reflecting rtt = " << rttMsg.data << std::endl;
            rttRxPublisher.publish(rttMsg);
        } 
        else
        {
            // std::cerr << "SMControlPlugin: reflectRTT: Did not receive RTT" << std::endl;
        }
        rttMutex.unlock();
    }

    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
        // std::cerr << "SMControlPlugin-" << getpid() << "::" << __func__ << ": Method Called!" << std::endl;

        if (isFirstSend)
        {
            char buffer[30];
            struct timeval tv;
            time_t curtime;

            gettimeofday(&tv, NULL); 
            curtime=tv.tv_sec;
          
            strftime(buffer,30,"%m-%d-%Y  %T.", localtime(&curtime));
            // printf("%s%ld\n",buffer,tv.tv_usec);

            std::cerr << "SMControlPlugin-" << getpid() << "::" << __func__ << ": Sending first state at time:"
                << buffer << tv.tv_usec << std::endl;

            isFirstSend = false;
        }

        // std::cerr << "SMControlPlugin (" << getpid() << "): Starting OnUpdate method." << std::endl;

        sendJointState();
        sendOdometry();

        // std::cerr << "SMControlPlugin (" << getpid() << "): Done sending joint state." << std::endl;
        receiveCmd();
        // std::cerr << "SMControlPlugin (" << getpid() << "): Done receiving command." << std::endl;
        reflectRTT();
        // std::cerr << "SMControlPlugin (" << getpid() << "): Ended OnUpdate method." << std::endl;
      
        // Prints the robot state being saved into shared memory
        // std::stringstream ss;
        // for(unsigned int ii = 0; ii < joint_index_sm_to_gazebo_map.size(); ii++)
        // {
        // size_t gazeboIndex = joint_index_sm_to_gazebo_map[ii];

        // ss << "  Joint: " << joints[gazeboIndex]->GetName() << ", position: " << positions[ii]
        //    << ", velocity: " << velocities[ii] << ", torque: " << torques[ii] << std::endl;
        // }
        // std::cerr << "SMControlPlugin: Sending Robot State:" << std::endl << ss.str();
    }

    // void idleLoop()
    // {

        // while(true)
        // {
        //     std::cerr << "SMControlPlugin (" << getpid() << "), " << __func__ 
        //         << ": Idle phase: Sending odometry, joint state, and RTT RX messages..." << std::endl;

        //     odometryPublisher.publish(odomMsg);
        //     robotStatePublisher.publish(jointStateMsg);
        //     rttRxPublisher.publish(rttMsg);

        //     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        // }
    // }

private:
      /*!
       * Pointer to the Gazebo robot model
       */
      physics::ModelPtr model;
  
      /*!
       * Pointer to the update event connection
       */
      event::ConnectionPtr updateConnection;
  
      /*!
       * Pointer to the handle that publishes robot joint state messages to
       * the shared memory.
       */
      shared_memory_interface::Publisher<sensor_msgs::JointState> robotStatePublisher;
  
     /*!
       * Pointer to the handle that publishes odometry state messages to
       * the shared memory.
       */
      shared_memory_interface::Publisher<nav_msgs::Odometry> odometryPublisher;
  
      /*!
       * Pointer to the handle that publishes RTT RX messages to
       * the shared memory.
       */
      shared_memory_interface::Publisher<std_msgs::Int64> rttRxPublisher;
  
      /*!
       * Pointer to the handle that receives command messages from shared memory.
       */
      shared_memory_interface::Subscriber<std_msgs::Float64MultiArray> cmdSubscriber;
  
      /*!
       * Pointer to the handle that receives RTT TX messages from shared memory.
       */
      shared_memory_interface::Subscriber<std_msgs::Int64> rttTxSubscriber;
  
      /*!
       * Maps joint index in shared memory to joint index in Gazebo.
       */
      std::vector<size_t> joint_index_sm_to_gazebo_map;
  
      /*!
       * Contains a 1 if the joint is masked b/c it is unactuated, 0 otherwise.
       * It is indexed in Gazebo joint order.
       */
      std::vector<size_t> unactuated_joint_mask;
  
      /*!
       * Pointer to a ROS node handle.
       */
      ros::NodeHandle * rosNode;
  
      /*!
       * The topic on which to publish odometry information
       */
      std::string rootLinkOdomTopic;
  
      /*!
       * Ensures only one thread can access rttMsg at a time.
       */
      std::mutex rttMutex;
  
      /*!
       * The message for relaying the RTT sequence number.
       */
      std_msgs::Int64 rttMsg;
  
      /*!
       * Ensures only one thread can access cmd at a time.
       */
      std::mutex cmdMutex;
  
      /*!
       * The message containing the torque command.
       */
      std_msgs::Float64MultiArray cmdMsg;
  
      /*!
       * Whether a command was received.
       */
      bool rcvdCmd;
  
      /*!
       * Whether a RTT message was received.
       */
      bool rcvdRTT;
  
      /*!
       * Holds the odometry state.  Used when writing the odometry state to shared memory.
       */
      nav_msgs::Odometry odomMsg;
  
      /*!
       * Holds the joint state.  This is used when writing the joint state to shared memory.
       */
      sensor_msgs::JointState jointStateMsg;

      ros::Time noCmdErrLastPrintTime;

      /*!
       * Whether this is the first time we're sending robot state.
       */
      bool isFirstSend;
      
      /*!
       * Whether the initial command message was received.
       */
      bool rcvdInitCmdMsg;

      /*!
       * Whether the initial RTT message was received.
       */
      bool rcvdInitRTTMsg;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(SMControlPlugin)

} // namespace gazebo
