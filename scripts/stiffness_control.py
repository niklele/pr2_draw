#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_draw')
import rospy
from tf.transformations import quaternion_from_euler as quaternion
import ee_cart_imped_action

import numpy as np
import numpy.linalg as la

MAX_LIN_STIFFNESS = 1000
MAX_ROT_STIFFNESS = 30

class StiffnessController(object):
    """contains helper functions for ee_cart_imped_action using stiffness control"""
    def __init__(self, arm='right_arm'):
        super(StiffnessController, self).__init__()
        self.arm = arm
        self.control = ee_cart_imped_action.EECartImpedClient(self.arm)

        self.home_pos = [0.75,0,0]
        self.home_orientation = quaternion(0, -np.pi/6, 0) # roll, pitch, yaw

        # ensure that time stamps move forward
        self.last_t = 0

        self.vel = 0.015 # not actual velocity because it is based on commanded position, not actual
        self.t = 5 # trajectory time starts ahead for a safety delay
        self.last_pos_cmd = self.home_pos

    def move(self, pos, sx):
        # calc time to travel based on velocity
        dt = la.norm(np.array(pos) - np.array(self.last_pos_cmd)) / self.vel

        # add a goal
        self.add_stiff_goal(pos, [sx, 1000, 1000], self.home_orientation, self.t + dt)

        # advance state
        self.last_pos_cmd = pos
        self.t += dt

    def home(self):
        """add goal to home near whiteboard"""
        self.move(self.home_pos, 50)

    def sendGoal(self):
        """wrap ee_cart_imped_action sendGoal"""
        self.control.sendGoal()

    def add_stiff_goal(self, pt, stiffness, orientation, t):
        """
        pt: 3-tuple set point in /torso_lift_link frame
        sx: 4-tuple set orientation in /torso_lift_link frame
        t: goal time
        """
        if t <= self.last_t:
            raise Exception('time must be later than the last time in the trajectory')
        else:
            self.last_t = t
            x, y, z = pt
            ox, oy, oz, ow = orientation
            sx, sy, sz = stiffness
            self.control.addTrajectoryPoint(x, y, z,
                                            ox, oy, oz, ow,
                                            sx, sy, sz,
                                            MAX_ROT_STIFFNESS, MAX_ROT_STIFFNESS, MAX_ROT_STIFFNESS,
                                            False, False, False,
                                            False, False, False,
                                            t,
                                            '/torso_lift_link')
