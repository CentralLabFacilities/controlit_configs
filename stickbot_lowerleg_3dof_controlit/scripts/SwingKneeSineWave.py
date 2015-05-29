#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Makes the knee move according to a sine wave.

import rospy
import sys, getopt     # for getting and parsing command line arguments and for exiting
import math            # for pi and various math functions
import string          # for string.join
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

DEFAULT_MIN_ANGLE = -math.pi
DEFAULT_MAX_ANGLE = math.pi
DEFAULT_PERIOD = 2              # in seconds

class KneeSwinger:
    def __init__(self, minAngle, maxAngle, period):
        """
        The constructor.

        Keyword arguments:
        minAngle -- The minimum angle of the knee in radians
        maxangle -- The maximum angle of the knee in radians
        period -- The period of the swing in seconds
        """

        self.minAngle = minAngle
        self.maxAngle = maxAngle
        self.period = period

        self.goalPositionPublisher = rospy.Publisher('/stickbot_lowerleg_3dof_controller/JPosTask/goalPosition', Float64MultiArray)

    def doSwing(self):

        GOAL_POSITION = [0.0, 0.6, 0.8]

        dim = MultiArrayDimension()
        dim.size = len(GOAL_POSITION)
        dim.label = "goalPosMsg"
        dim.stride = len(GOAL_POSITION)

        goalPositionMsg = Float64MultiArray()
        goalPositionMsg.data = GOAL_POSITION
        goalPositionMsg.layout.dim.append(dim)
        goalPositionMsg.layout.data_offset = 0

        loopCounter = 0  # Temporary!
        loopClock = rospy.Rate(1.0 / self.period)

        while not rospy.is_shutdown():

            if loopCounter % 2 == 0:                  # Temporary!
                GOAL_POSITION = [0.0, 0.3, -0.5]      # Temporary!
            else:                                     # Temporary!
                GOAL_POSITION = [0.0, 0.6, 0.8]       # Temporary!

            goalPositionMsg.data = GOAL_POSITION

            rospy.loginfo("Publishing goal: {0}".format(map(str, goalPositionMsg.data)))
            self.goalPositionPublisher.publish(goalPositionMsg)

            loopCounter = loopCounter + 1  # Temporary!
            loopClock.sleep()

# Main method
if __name__ == "__main__":

    rospy.init_node('KneeSwinger', anonymous=True)

    # Define default values for the command line arguments
    minAngle = DEFAULT_MIN_ANGLE
    maxAngle = DEFAULT_MAX_ANGLE
    period = DEFAULT_PERIOD

    usageStr = string.join(["python SwingKnee.py [parameters]\nValid parameters include:\n",
        " -h Prints this help message\n",
        " -l or --minAngle [min angle] (default is {0})\n".format(minAngle),
        " -u or --maxAngle [max angle] (default is {0})\n".format(maxAngle),
        " -p or --period [period] (default is {0})".format(period)])

    # Parse the command line arguments
    try:
        opts, args = getopt.getopt(sys.argv[1:],"hl:u:p:",["minAngle=", "maxangle=", "period="])
    except getopt.GetoptError:
       print usageStr
       sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            print usageStr
            sys.exit()
        elif opt in ("-l", "--minAngle"):
            minAngle = float(arg)
        elif opt in ("-u", "--maxAngle"):
            maxAngle = float(arg)
        elif opt in ("-p", "--period"):
            period = float(arg)
        else:
            print "Unknown argument \"{0}\"".format(opt)

    rospy.loginfo("main: Started with the following arguments:\n"
                  "  - minAngle: {0}\n".format(minAngle) +
                  "  - maxAngle: {0}\n".format(maxAngle) +
                  "  - period: {0}\n".format(period))

    # Create a DatabaseInterface object
    kneeSwinger = KneeSwinger(minAngle, maxAngle, period)
    try:
        kneeSwinger.doSwing()
    except rospy.ROSInterruptException: pass
