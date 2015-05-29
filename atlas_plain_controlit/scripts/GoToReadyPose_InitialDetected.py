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
from rapid_core.srv import get_parameters
from diagnostic_msgs.msg import DiagnosticStatus
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

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

        # Create a subscriber to get the initial state of the robot
        self.gotJointState = False
        self.currPosSubscriber = rospy.Subscriber("/joint_states", JointState, self.jointStateCallback)

        # Get the joint indices
        self.gotJointIndices = False
        rospy.wait_for_service(JOINT_INDICES_SERVICE)
        getJointIndicesProxy = rospy.ServiceProxy(JOINT_INDICES_SERVICE, get_parameters)
        try:
            self.jointIndices = getJointIndicesProxy()
            self.gotJointIndices = True
        except rospy.ServiceException as exc:
            rospy.logerr("Failed to call service {0}.  Did not get joint indices.".format(JOINT_INDICES_SERVICE))


    def start(self):
        """
        Starts the trajectory generation.
        """

        # Only proceed if we're able to get the joint indices
        if not self.gotJointIndices:
            print "Did not get joint indices.  aborting."
            return;

        # Pause for the required number of seconds
        rospy.loginfo("Pausing for {0} seconds.".format(self.delay))
        rospy.sleep(self.delay)
        rospy.loginfo("Starting to send trajectory...")

        # Wait for the initial joint state information to arrive
        waitCnt = 0
        while not self.gotJointState and waitCnt < 10:
            rospy.loginfo("Did not get initial joint state.  Waiting 1 second.")
            waitCnt = waitCnt + 1
            rospy.sleep(1)

        # Create the message

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

        # Determine the initial joint state
        for joint in self.jointIndices.params.values:

            indx = int(joint.key)
            currAngle = self.jointStateMsg.position[indx]

            # rospy.loginfo("Joint {0} at index {1} is initially at {2}.".format(joint.value, joint.key, currAngle))
            goalPosMsg.data[indx] = currAngle

        done = False

        # Here begins the loop that generates the trajectory over time
        while not done and not rospy.is_shutdown():

            done = True

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

            self.goalPosPublisher.publish(goalPosMsg)

            if not done:
                # print "Sleeping for {0} seconds.".format(self.timeStep)
                rospy.sleep(self.timeStep)
                # time.sleep(self.timeStep)

        rospy.loginfo("Done sending trajectory.")

    def jointStateCallback(self, data):
        """
        The callback function for the joint state message.
        """
        self.jointStateMsg = data
        self.gotJointState = True

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