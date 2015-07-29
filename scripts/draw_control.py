#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_draw')
import rospy
# from tf.transformations import quaternion_from_euler as quaternion
import tf
import geometry_msgs
import ee_cart_imped_action

import numpy as np
import numpy.linalg as la

MAX_LIN_STIFFNESS = 1000
MAX_ROT_STIFFNESS = 30

class DrawController(object):
    """contains helper functions for ee_cart_imped_action"""
    def __init__(self, stiffness_control, arm='right_arm'):
        super(DrawController, self).__init__()
        self.stiffness_control = stiffness_control
        self.arm = arm
        self.control = ee_cart_imped_action.EECartImpedClient('right_arm')
        self.tf_listener = tf.TransformListener()

        self.tf_echo_sub = rospy.Subscriber("/tf_echo/torso_lift_link/r_gripper_tool_frame", geometry_msgs.msg.Transform, self.tf_echo_cb)
        self.curr_pos = []

        self.home_pos = [0.75,0,0]
        self.home_orientation = tf.transformations.quaternion_from_euler(0, -np.pi/6, 0) # roll, pitch, yaw

        # ensure that time stamps move forward
        self.last_t = 0

        self.vel = 0.015 # not actual velocity because it is based on commanded position, not actual
        self.t = 5 # trajectory time starts ahead for a safety delay
        self.last_pos_cmd = self.home_pos
        self.last_orientation_cmd = [0,0,0,1]

    def tf_echo_cb(self, data):
        self.curr_pos = [data.translation.x, data.translation.y, data.translation.z]

    # def curr_pos(self):
    #     """
    #     use tf to get current position of gripper in /torso_lift_link frame.
    #     does not return orientation
    #     """
    #
    #     TF method
    #     while(True):
    #         try:
    #             # TODO check order
    #             position, orientation = self.tf_listener.lookupTransform('/torso_lift_link',
    #                                                                      '/r_gripper_tool_frame', rospy.Time(0))
    #             return position
    #         except Exception as e:
    #             # raise(Exception("tf error in curr_pos: " + str(e)))
    #             rospy.logwarn("tf error in curr_pos: " + str(e))

    def move(self, pos, orientation, fx):
        """add a goal to move to the position and orientation with force/stiffness fx in x-axis
        pos: position x,y,z in /torso_lift_link frame
        orientation: ox,oy,oz,ow in /torso_lift_link frame
        """

        # calc time to travel based on velocity
        dt = la.norm(np.array(pos) - np.array(self.last_pos_cmd)) / self.vel

        dt += 0.001 # add small amount

        # add a goal
        self.add_goal(pos,
                      [fx, MAX_LIN_STIFFNESS, MAX_LIN_STIFFNESS],
                      orientation,
                      self.t + dt)

        # advance state
        self.last_pos_cmd = pos
        self.last_orientation_cmd = orientation
        self.t += dt

    def loiter(self, time):
        """add a goal to stay at the last command for a certain amount of time"""
        self.add_goal(self.last_pos_cmd,
                      [100, MAX_LIN_STIFFNESS, MAX_LIN_STIFFNESS],
                      self.last_orientation_cmd,
                      self.t + time)
        self.t += time


    def home(self):
        """add goal to home near whiteboard"""
        self.move(self.home_pos, [0,0,0,1], 50)

    def sendGoal(self):
        """wrap ee_cart_imped_action sendGoal"""
        self.control.sendGoal()

    def add_goal(self, pt, force, orientation, t):
        """
        pt: set point x,y,z in /torso_lift_link frame
        force: force/stiffness in x,y,z
        orientation: set orientation ox,oy,oz,ow in /torso_lift_link frame
        t: goal time
        """
        if t <= self.last_t:
            raise Exception('time ({0}) must be later than the last time ({1}) in the trajectory'.format(t, self.last_t))
        # else if not self.stiffness_control:
        #     if
        else:
            self.last_t = t
            x, y, z = pt
            ox, oy, oz, ow = orientation
            fx, fy, fz = force

            if (self.stiffness_control):
                self.control.addTrajectoryPoint(x, y, z,
                                                ox, oy, oz, ow,
                                                fx, fy, fz,
                                                MAX_ROT_STIFFNESS, MAX_ROT_STIFFNESS, MAX_ROT_STIFFNESS,
                                                False, False, False,
                                                False, False, False,
                                                t,
                                                '/torso_lift_link')
            else:
                self.control.addTrajectoryPoint(x, y, z,
                                                ox, oy, oz, ow,
                                                fx, fy, fz,
                                                MAX_ROT_STIFFNESS, MAX_ROT_STIFFNESS, MAX_ROT_STIFFNESS,
                                                True, False, False,
                                                False, False, False,
                                                t,
                                                '/torso_lift_link')
