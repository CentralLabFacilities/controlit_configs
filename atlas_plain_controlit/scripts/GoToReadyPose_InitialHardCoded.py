#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Generates a sequence of reference positions that moves atlas_plain to a starting pose.
'''

import sys, getopt     # for getting and parsing command line arguments
import time
import math
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

# Goal Position:
#   0.0,   # back_bkz
#   0.02,  # back_bky
#   0.0,   # back_bkx
#   0.0,   # l_arm_shy
#   -1.5,  # l_arm_shx
#   1.5,   # l_arm_ely
#   1.4,   # l_arm_elx
#   0.0,   # l_arm_wry
#   0.0,   # l_arm_wrx
#   0.0,   # neck_ry
#   0.0,   # r_arm_shy
#   1.5,   # r_arm_shx
#   1.5,   # r_arm_ely
#   -1.4,  # r_arm_elx
#   0.0,   # r_arm_wry
#   0.0,   # r_arm_wrx
#   0.0,   # l_leg_hpz
#   0.0,   # l_leg_hpx
#   0.0,   # l_leg_hpy
#   0.18,  # l_leg_kny
#   -0.15, # l_leg_aky
#   0.0,   # l_leg_akx
#   0.0,   # r_leg_hpz
#   0.0,   # r_leg_hpx
#   0.0,   # r_leg_hpy
#   0.18,  # r_leg_kny
#   -0.15, # r_leg_aky
#   0.0]   # r_leg_akx

INITIAL_POSITION = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
GOAL_POSITION = [0.0, 0.02, 0.0, 0.0, -1.5, 1.5, 1.4, 0.0, 0.0, 0.0, 0.0, 1.5, 1.5, -1.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.18, -0.15, 0.0, 0.0, 0.0, 0.0, 0.18, -0.15, 0.0]

JOINT_INDICES_SERVICE = "/atlas_plain_controller/diagnostics/getJointIndices"

class GoToReadyPose:
    def __init__(self, maxStepSize, timeStep, delay):
        """
        The constructor.

        Keyword arguments:
        maxStepSize -- The maximum change in joint angle that can occur during each time step.
        timeStep -- The time step size in seconds.
        delay -- The amount of time to wait in seconds prior to starting.  This allows other components to spin up and the robot to stabilize prior to moving.
        """

        self.maxStepSize = maxStepSize
        self.timeStep = timeStep
        self.delay = delay

        # Create the publisher
        self.goalPosPublisher = rospy.Publisher("/atlas_plain_controller/JPosTask/goalPosition", Float64MultiArray)

    def start(self):
        """
        Starts the trajectory generation.
        """

        # Pause for the required number of seconds
        rospy.loginfo("Pausing for {0} seconds.".format(self.delay))
        rospy.sleep(self.delay)
        rospy.loginfo("Starting to send trajectory...")

        # Define the dimensions of the message
        dim = MultiArrayDimension()
        dim.size = len(GOAL_POSITION)
        dim.label = "goalPosMsg"
        dim.stride = len(GOAL_POSITION)

        goalPosMsg = Float64MultiArray()
        for ii in range(0, len(GOAL_POSITION)):
            goalPosMsg.data.append(0)
        goalPosMsg.layout.dim.append(dim)
        goalPosMsg.layout.data_offset = 0

        for index in range(len(INITIAL_POSITION)):
            # rospy.loginfo("Joint at index {0} is initially at {1}.".format(index, INITIAL_POSITION[index]))
            goalPosMsg.data[index] = INITIAL_POSITION[index]

        done = False

        # Here begins the loop that generates the trajectory over time
        while not done and not rospy.is_shutdown():

            done = True
            # print "Adjusting joint references:"

            # Update the goal position
            for indx in range(len(GOAL_POSITION)):
                # print "Checking joint {0}, currGoal: {1}, finalGoal: {2}".format(indx, goalPosMsg.data[indx], GOAL_POSITION[indx])
                if not goalPosMsg.data[indx] == GOAL_POSITION[indx]:
                    done = False
                    delta = goalPosMsg.data[indx] - GOAL_POSITION[indx]

                    if  math.fabs(delta) > self.maxStepSize:
                        if delta > 0:
                            goalPosMsg.data[indx] = goalPosMsg.data[indx] - self.maxStepSize
                        else:
                            goalPosMsg.data[indx] = goalPosMsg.data[indx] + self.maxStepSize
                    else:
                        goalPosMsg.data[indx] = GOAL_POSITION[indx]

                    # print " - joint {0}, currGoal: {1}, finalGoal: {2}".format(indx, goalPosMsg.data[indx], GOAL_POSITION[indx])

            self.goalPosPublisher.publish(goalPosMsg)

            if not done:
                # print "Sleeping for {0} seconds.".format(self.timeStep)
                rospy.sleep(self.timeStep)
                # time.sleep(self.timeStep)

        rospy.loginfo("Done sending trajectory.")

# Main method
if __name__ == "__main__":

    rospy.init_node('GoToReadyPose', anonymous=True)

    # Define default values for the command line arguments
    maxStepSize = 0.005  # in radians
    timeStep = 0.01      # in seconds
    delay = 5.0          # in seconds

    usageStr = "Usage: python GoToReadyPose.py [parameters]\nValid parameters include:\n -s or --maxStepSize [max step size in radians] (default {0})\n -t or --timeStep [time step in seconds] (default {1})\n -d or --delay [delay in seconds] (default {2})".format(
        maxStepSize, timeStep, delay)

    # Parse the command line arguments
    try:
        opts, args = getopt.getopt(sys.argv[1:],"hs:t:d:",["maxStepSize=", "timeStep=", "delay="])
    except getopt.GetoptError:
       rospy.logerr(usageStr)
       sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            rospy.loginfo(usageStr)
            sys.exit()
        elif opt in ("-s", "--maxStepSize"):
            maxStepSize = float(arg)
        elif opt in ("-t", "--timeStep"):
            timeStep = float(arg)
        elif opt in ("-d", "--delay"):
            delay = float(arg)
        else:
            print "Unknown argument \"{0}\"".format(opt)

    rospy.loginfo("Started with the following arguments:\n  - max step size {0}\n  - time step: {1}\n  - delay: {2}".format(
        maxStepSize, timeStep, delay))

    # Create a GoToReadyPose object
    GoToReadyPose = GoToReadyPose(maxStepSize, timeStep, delay)
    GoToReadyPose.start()