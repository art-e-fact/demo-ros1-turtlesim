#!/usr/bin/env python3
import sys
import unittest
from std_srvs.srv import Empty as EmptySrv
from turtlesim.srv import TeleportAbsolute, SetPen
from turtlesim.msg import Pose as TurtlePose
import rospy
import numpy as np
from turtle_trajectory import TurtleTrajectory

PKG = 'turtle_simple'
NAME = 'turtle1'
START_POSE = {'x': 1, 'y': 1, 'theta': 0}
SEGMENT_LENGTH = 5
SPEED = 5


class TestTurtle(unittest.TestCase):

    # runs before each test_* method
    def setUp(self):
        ## Arange
        rospy.init_node("testturtlesimple", anonymous=True)
        # setup the turtle in the simulator
        rospy.wait_for_service(f"/{NAME}/teleport_absolute")
        rospy.wait_for_service(f"/clear")
        self.srv_teleport_absolute = rospy.ServiceProxy(f"/{NAME}/teleport_absolute", TeleportAbsolute)
        self.srv_clear = rospy.ServiceProxy(f"/clear", EmptySrv)
        self.srv_teleport_absolute(
            x=START_POSE['x'],
            y=START_POSE['y'],
            theta=START_POSE['theta']
        )
        self.srv_clear()
        # prepare trajectory commands for the test
        self.turtle_trajectory = TurtleTrajectory(NAME, START_POSE)

    def test_turtle(self):
        ## Act
        self.turtle_trajectory.closed_loop_square(
            speed=SPEED,
            segment_length=SEGMENT_LENGTH
        ) # drive the turtle!
        self.turtle_trajectory.stop_commands()

        ## Assert
        final_pose = rospy.wait_for_message(
            f'/{NAME}/pose',
            TurtlePose,
            timeout=None
        )
        distance_to_start = np.sqrt((final_pose.x - START_POSE['x'])**2 + (final_pose.y - START_POSE['y'])**2)

        # check if the turtle moved (= is not at exactly the starting position)
        self.assertNotEqual(
            distance_to_start,
            0,
            msg=f"turtle position at end of test exactly at starting position: did not move?"
        )
        # check if the turtle finished the loop trajectory
        self.assertAlmostEqual(
            distance_to_start,
            0,
            delta=0.5,
            msg=f"turtle position at end of test = {distance_to_start}m is more than 0.5m away from starting position: error in trajectory?"
        )


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestTurtle, sys.argv)
