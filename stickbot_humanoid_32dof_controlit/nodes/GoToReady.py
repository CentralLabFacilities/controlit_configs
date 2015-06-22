#!/usr/bin/env python

'''
Makes stickbot_humanoid_32dof_controlit go into a ready state.
'''

# import sys, getopt     # for getting and parsing command line arguments
import time
# import math
# import threading
import rospy

from std_msgs.msg import Float64MultiArray, MultiArrayDimension, Int32
from controlit_trajecotry_generators import TrapezoidVelocityTrajGen

jPosStart = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,]

jPosEnd = [0.0, -0.3, 0.0, 0.65, -0.35, 0.0,    # left leg
           0.0, -0.3, 0.0, 0.65, -0.35, 0.0,    # right leg
           0.0, 0.0, 0.0,                       # torso
           0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0,   # left arm
           0.0, 0.0, 0.0,                       # neck
           0.0, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0]  # right arm

class GoToReady:
    def __init__(self):

        self.publisher = rospy.Publisher("/stickbot_humanoid_32dof_controller/JPosTask/goalPosition",
            Float64MultiArray, queue_size=1)

    def run(self):
        """
        Makes the robot go to the ready position.
        """

        # Define the dimensions of the message
        dim = MultiArrayDimension()
        dim.size = len(jPosStart)
        dim.label = "goalMsg"
        dim.stride = 1

        # Define the goal message
        self.goalMsg = Float64MultiArray()
        for ii in range(0, len(jPosStart)):
            self.goalMsg.data.append(jPosStart[ii])
        self.goalMsg.layout.dim.append(dim)
        self.goalMsg.layout.data_offset = 0

        rate = rospy.Rate(100) # 100Hz
        pctDone = 0
        while not rospy.is_shutdown() and pctDone < 1.0:

            pctDone = pctDone + 0.01
            rate.sleep()


# Main method
if __name__ == "__main__":

    rospy.init_node('GoToReady', anonymous=True)

    goToReady = GoToReady()
    goToReady.run()
