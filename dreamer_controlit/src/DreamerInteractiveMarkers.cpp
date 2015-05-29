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

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <geometry_msgs/Pose.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

using visualization_msgs::Marker;
using visualization_msgs::InteractiveMarker;
using visualization_msgs::InteractiveMarkerControl;
using visualization_msgs::InteractiveMarkerFeedbackConstPtr;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;

bool rcvdRightWristPosition = false, rcvdRightWristOrientation = false,
     rcvdLeftWristPosition = false, rcvdLeftWristOrientation = false;

bool mousedown = false;

tf::Vector3 rightWristPosition, leftWristPosition;
tf::Quaternion rightWristOrientation, leftWristOrientation;

ros::Publisher rightWristPosPub, rightWristOrientationPub,
               leftWristPosPub,  leftWristOrientationPub;

// %Tag(Box)%
Marker makeBox(InteractiveMarker &msg)
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.1;
  marker.scale.y = msg.scale * 0.1;
  marker.scale.z = msg.scale * 0.1;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

InteractiveMarkerControl& makeBoxControl(InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}
// %EndTag(Box)%

void processFeedback(
    const InteractiveMarkerFeedbackConstPtr &feedback )
{
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  // std::ostringstream mouse_point_ss;
  // if( feedback->mouse_point_valid )
  // {
  //   mouse_point_ss << " at " << feedback->mouse_point.x
  //                  << ", " << feedback->mouse_point.y
  //                  << ", " << feedback->mouse_point.z
  //                  << " in frame " << feedback->header.frame_id;
  // }

  // These are used to publish the new orientation goal
  std_msgs::MultiArrayDimension dimOriMsg, dimPosMsg;
  std_msgs::Float64MultiArray oriMsg, posMsg;

  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      // ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      // ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      ROS_INFO_STREAM( s.str() << ": pose changed"
          << "\nposition (x, y, z) = "
          << feedback->pose.position.x
          << ", " << feedback->pose.position.y
          << ", " << feedback->pose.position.z
          << "\norientation (w, x, y, z) = "
          << feedback->pose.orientation.w
          << ", " << feedback->pose.orientation.x
          << ", " << feedback->pose.orientation.y
          << ", " << feedback->pose.orientation.z
          << "\nframe: " << feedback->header.frame_id
          << " time: " << feedback->header.stamp.sec << "sec, "
          << feedback->header.stamp.nsec << " nsec" );

      dimOriMsg.size = 4;
      dimOriMsg.stride = 4;

      oriMsg.layout.data_offset = 0;
      oriMsg.layout.dim.push_back(dimOriMsg);
      oriMsg.data.resize(4);
      oriMsg.data[0] = feedback->pose.orientation.w;
      oriMsg.data[1] = feedback->pose.orientation.x;
      oriMsg.data[2] = feedback->pose.orientation.y;
      oriMsg.data[3] = feedback->pose.orientation.z;

      dimPosMsg.size = 3;
      dimPosMsg.stride = 3;

      posMsg.layout.data_offset = 0;
      posMsg.layout.dim.push_back(dimPosMsg);
      posMsg.data.resize(3);
      posMsg.data[0] = feedback->pose.position.x;
      posMsg.data[1] = feedback->pose.position.y;
      posMsg.data[2] = feedback->pose.position.z;

      if (feedback->marker_name.compare("dreamer_right_wrist_6dof") == 0)
      {
        rightWristOrientationPub.publish(oriMsg);
        rightWristPosPub.publish(posMsg);  
      }
      else
      {
        leftWristOrientationPub.publish(oriMsg);
        leftWristPosPub.publish(posMsg);
      }
      

      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      // ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
      mousedown = true;
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      // ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
      mousedown = false;
      break;
  }

  server->applyChanges();
}

void rightWristPosCallback(const boost::shared_ptr<std_msgs::Float64MultiArray const> & msgPtr)
{
  // ROS_INFO("I heard: [%s]", msg->data.c_str());
  rightWristPosition[0] = msgPtr->data[0];
  rightWristPosition[1] = msgPtr->data[1];
  rightWristPosition[2] = msgPtr->data[2];
  rcvdRightWristPosition = true;
}

void rightWristOriCallback(const boost::shared_ptr<std_msgs::Float64MultiArray const> & msgPtr)
{
  // Note order differences:
  //  -  writeWristOrientation: x, y, z, w
  //  -  msgPtr: w, x, y, z
  rightWristOrientation[0] = msgPtr->data[1]; // x
  rightWristOrientation[1] = msgPtr->data[2]; // y
  rightWristOrientation[2] = msgPtr->data[3]; // z
  rightWristOrientation[3] = msgPtr->data[0]; // w

  rcvdRightWristOrientation = true;
}

void leftWristPosCallback(const boost::shared_ptr<std_msgs::Float64MultiArray const> & msgPtr)
{
  // ROS_INFO("I heard: [%s]", msg->data.c_str());
  leftWristPosition[0] = msgPtr->data[0];
  leftWristPosition[1] = msgPtr->data[1];
  leftWristPosition[2] = msgPtr->data[2];
  rcvdLeftWristPosition = true;
}

void leftWristOriCallback(const boost::shared_ptr<std_msgs::Float64MultiArray const> & msgPtr)
{
  // Note order differences:
  //  -  writeWristOrientation: x, y, z, w
  //  -  msgPtr: w, x, y, z
  leftWristOrientation[0] = msgPtr->data[1]; // x
  leftWristOrientation[1] = msgPtr->data[2]; // y
  leftWristOrientation[2] = msgPtr->data[3]; // z
  leftWristOrientation[3] = msgPtr->data[0]; // w

  rcvdLeftWristOrientation = true;
}

void create6DOFMarker(std::string name, std::string description,
  tf::Vector3 & position, tf::Quaternion & orientation)
{
  InteractiveMarker marker;
  marker.header.frame_id = "world"; //"base_link";
  tf::pointTFToMsg(position, marker.pose.position);
  tf::quaternionTFToMsg(orientation, marker.pose.orientation);
  marker.scale = 0.2;

  marker.name = name;
  marker.description = description; 

  // insert a box
  makeBoxControl(marker);
  marker.controls[0].interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;

  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  marker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  marker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(control);

  server->insert(marker);
  server->setCallback(marker.name, &processFeedback);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "controlit_dreamer_interactive_marker");

  ros::NodeHandle nh;

  // Create publishers for the new position and quaternion goals
  rightWristOrientationPub = nh.advertise<std_msgs::Float64MultiArray>("/dreamer_controller/RightHandOrientation/goalOrientation", 1000);
  rightWristPosPub = nh.advertise<std_msgs::Float64MultiArray>("/dreamer_controller/RightHandPosition/goalPosition", 1000);
  leftWristOrientationPub = nh.advertise<std_msgs::Float64MultiArray>("/dreamer_controller/LeftHandOrientation/goalOrientation", 1000);
  leftWristPosPub = nh.advertise<std_msgs::Float64MultiArray>("/dreamer_controller/LeftHandPosition/goalPosition", 1000);

  // Subscribe to the current position and orientation of the right wrist
  ros::Subscriber subRightWristPosition = nh.subscribe("/dreamer_controller/RightHandPosition/actualWorldPosition", 1000, rightWristPosCallback);
  ros::Subscriber subRightWristOrientation = nh.subscribe("/dreamer_controller/RightHandOrientation/actualWorldOrientation", 1000, rightWristOriCallback);
  ros::Subscriber subLeftWristPosition = nh.subscribe("/dreamer_controller/LeftHandPosition/actualWorldPosition", 1000, leftWristPosCallback);
  ros::Subscriber subLeftWristOrientation = nh.subscribe("/dreamer_controller/LeftHandOrientation/actualWorldOrientation", 1000, leftWristOriCallback);

  ros::Rate loop_rate(10);
  while (ros::ok() && (!rcvdRightWristPosition || !rcvdRightWristOrientation || !rcvdLeftWristPosition || !rcvdLeftWristOrientation))
  {
    std::stringstream buff;
    buff << "Waiting for the following state:";
    if (!rcvdRightWristPosition)
      buff << "\n  - right wrist position";

    if (!rcvdRightWristOrientation)
      buff << "\n  - right wrist orientation";

    if (!rcvdLeftWristPosition)
      buff << "\n  - left wrist position.";

    if (!rcvdLeftWristOrientation)
      buff << "\n  - left wrist orientation.";


    ROS_INFO_STREAM(buff.str());
    ros::spinOnce();
    loop_rate.sleep();
  }

  // ROS_INFO_STREAM("Got right hand position: [" << rightWristPosition[0] << ", " << rightWristPosition[1] << ", " << rightWristPosition[2] << "]");

  // create an interactive marker server on the topic namespace controlit_interactive_marker
  server.reset(new interactive_markers::InteractiveMarkerServer("controlit_dreamer_interactive_marker","",false));

  // tf::Vector3 position;
  // position = tf::Vector3(0, 0, 0);


  create6DOFMarker("dreamer_right_wrist_6dof", "Dreamer right wrist 6-DOF control",
    rightWristPosition, rightWristOrientation);
  create6DOFMarker("dreamer_left_wrist_6dof", "Dreamer left wrist 6-DOF control",
    leftWristPosition, leftWristOrientation);


  // menu_handler.insert("first entry", &processFeedback);
  // menu_handler.insert("second entry", &processFeedback);

  // menu_handler.apply(*server, int_marker.name);

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  // server->insert(int_marker, &processFeedback);

  // 'commit' changes and send to all clients
  server->applyChanges();

  // start the ROS main loop
  // ros::spin();

  
  while (ros::ok())
  {
    // static tf::TransformBroadcaster br;
    // tf::Transform transform;
    // transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    // tf::Quaternion q;
    // q.setRPY(0, 0, 0);
    // transform.setRotation(q);
    // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "/base_link"));

    if (!mousedown)
    {
      geometry_msgs::Pose right_wrist_pose, left_wrist_pose;

      right_wrist_pose.position.x = rightWristPosition[0];
      right_wrist_pose.position.y = rightWristPosition[1];
      right_wrist_pose.position.z = rightWristPosition[2];

      right_wrist_pose.orientation.x = rightWristOrientation[0];
      right_wrist_pose.orientation.y = rightWristOrientation[1];
      right_wrist_pose.orientation.z = rightWristOrientation[2];
      right_wrist_pose.orientation.w = rightWristOrientation[3];

      left_wrist_pose.position.x = leftWristPosition[0];
      left_wrist_pose.position.y = leftWristPosition[1];
      left_wrist_pose.position.z = leftWristPosition[2];

      left_wrist_pose.orientation.x = leftWristOrientation[0];
      left_wrist_pose.orientation.y = leftWristOrientation[1];
      left_wrist_pose.orientation.z = leftWristOrientation[2];
      left_wrist_pose.orientation.w = leftWristOrientation[3];

      if (!server->setPose("dreamer_right_wrist_6dof", right_wrist_pose))
        ROS_ERROR("Problems updating right wrist's interactive marker");
      if (!server->setPose("dreamer_left_wrist_6dof", left_wrist_pose))
        ROS_ERROR("Problems updating left wrist's interactive marker");

      server->applyChanges();

    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  server.reset();  // deallocates the memory

}