#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Sends a command to the Sandia hand
'''

import sys, getopt     # for getting and parsing command line arguments
import time
import math
import rospy
from osrf_msgs.msg import JointCommands

LEFT_HAND_COMMAND_TOPIC  = "/sandia_hands/l_hand/joint_commands"
RIGHT_HAND_COMMAND_TOPIC = "/sandia_hands/r_hand/joint_commands"

class SendSandiaHandCommand:
    def __init__(self):
        """
        The constructor.
        """

        # Create the publisher
        self.rightPublisher = rospy.Publisher(RIGHT_HAND_COMMAND_TOPIC, JointCommands)
        self.leftPublisher = rospy.Publisher(LEFT_HAND_COMMAND_TOPIC, JointCommands)

    def start(self):
        """
        Sends the command
        """

        # Some useful vectors
        vector0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        vector1 = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        vector5 = [5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5]
        vector20 = [20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20]
        vector100 = [100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100]
        vector500 = [500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500]

        # left hand:
        # index 0: pointer finger, 1 moves towards middle finger
        # index 1: pointer finger, 1 moves towards palm
        # index 2: pointer finger, 1 moves towards palm
        # index 3: middle finger, 1 moves towards pinky
        # index 4: middle finger, 1 moves towards palm
        # index 5: middle finger, 1 moves towards palm
        # index 6: pinky finger, 1 moves towards away from middle finger
        # index 7: pinky finger, 1 moves towards palm
        # index 8: pinky finger, 1 moves towards palm
        # index 9: thumb finger, 1 moves towards fingers
        # index 10: thumb finger, 1 moves towards fingers
        # index 11: thumb finger, 1 moves towards towards fingers

        leftCommand = JointCommands()
        leftCommand.name = ['left_f0_j0', 'left_f0_j1', 'left_f0_j2', 'left_f1_j0', 'left_f1_j1', 'left_f1_j2', 'left_f2_j0', 'left_f2_j1', 'left_f2_j2', 'left_f3_j0', 'left_f3_j1', 'left_f3_j2']
        leftCommand.position = [0.0, 1.0, 1.5, 0.0, 1.0, 1.5, 0.0, 1.0, 1.5, 0.5, 0.4, 1.0]
        leftCommand.velocity = vector0
        leftCommand.effort = vector0
        leftCommand.kp_position = vector500   # position error gain
        leftCommand.ki_position = vector1      # sum of position error gain
        leftCommand.kd_position = vector0      # change in position error over time gain
        leftCommand.kp_velocity = vector0   # velocity error gain
        leftCommand.i_effort_min = [-1000, -1000, -1000, -1000, -1000, -1000, -1000, -1000, -1000, -1000, -1000]
        leftCommand.i_effort_max = [1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000]

        rightCommand = JointCommands()
        rightCommand.name = ['right_f0_j0', 'right_f0_j1', 'right_f0_j2', 'right_f1_j0', 'right_f1_j1', 'right_f1_j2', 'right_f2_j0', 'right_f2_j1', 'right_f2_j2', 'right_f3_j0', 'right_f3_j1', 'right_f3_j2']
        rightCommand.position = [0.0, 1.0, 1.5, 0.0, 1.0, 1.5, 0.0, 1.0, 1.5, 0.5, 0.4, 1.0]
        rightCommand.velocity = vector0
        rightCommand.effort = vector0
        rightCommand.kp_position = vector500   # position error gain
        rightCommand.ki_position = vector1      # sum of position error gain
        rightCommand.kd_position = vector0      # change in position error over time gain
        rightCommand.kp_velocity = vector0    # velocity error gain
        rightCommand.i_effort_min = [-1000, -1000, -1000, -1000, -1000, -1000, -1000, -1000, -1000, -1000, -1000]
        rightCommand.i_effort_max = [1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000]

        count = 0
        while not rospy.is_shutdown() and count < 5:
            rospy.loginfo("Sending command, attempt {0} of {1}.".format(count+1, 5))

            rightCommand.header.stamp = rospy.Time.now()
            self.rightPublisher.publish(rightCommand)

            leftCommand.header.stamp = rospy.Time.now()
            self.leftPublisher.publish(leftCommand)

            count = count + 1

            if count < 5:
                rospy.sleep(0.5)

        rospy.loginfo("Done sending hand commands.")

# Main method
if __name__ == "__main__":

    rospy.init_node('SandiaHandCommandSender', anonymous=True)

    # Create a SendSandiaHandCommand object
    SendSandiaHandCommand = SendSandiaHandCommand()
    SendSandiaHandCommand.start()