#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_draw')
import rospy
import geometry_msgs
import ee_cart_imped_action
import tf

import numpy as np
import numpy.linalg as la

MAX_LIN_STIFFNESS = 1000
MAX_LIN_FORCE = 10
MAX_ROT_STIFFNESS = 30

class DrawController(object):
    """contains helper functions for ee_cart_imped_action using right arm"""
    def __init__(self):
        super(DrawController, self).__init__()
        self.control = ee_cart_imped_action.EECartImpedClient('right_arm')
        self.tf_echo_sub = rospy.Subscriber("/tf_echo/torso_lift_link/r_gripper_tool_frame", geometry_msgs.msg.Transform, self.tf_echo_cb)

        self.curr_pos = []
        self.curr_orientation = []
        self.t = 5 # trajectory time starts ahead for a safety delay

        self.home_pos = [0.75, 0, 0] # in /torso_lift_link
        self.home_orientation = tf.transformations.quaternion_from_euler(0, -np.pi/6, 0) # roll, pitch, yaw

        self.last_pos_cmd = self.home_pos
        self.last_orientation_cmd = [0,0,0,1]
        self.last_t = 0 # ensure that time stamps move forward


        self.vel = 0.015 # not actual velocity because it is based on commanded position, not actual
        self.rot_vel = np.pi/2 # radians/s

    def tf_echo_cb(self, data):
        """callback from tf_echo_sub sets current position and orientation"""
        self.curr_pos = [data.translation.x, data.translation.y, data.translation.z]
        self.curr_orientation = [data.rotation.x, data.rotation.y, data.rotation.z, data.rotation.w]

    def add_path_goals(self, path, stiffness):
        """send a path specified as a list of [position, orientation] with a given stiffness"""
        for position, orientation in path:
            # rospy.loginfo("Adding goal pos:{0} orientation:{1}".format(position, orientation))
            self.add_move_goal(position, orientation, stiffness)

    def add_move_goal(self, pos, orientation, sx=MAX_LIN_STIFFNESS):
        """add a goal to move to the position and orientation with stiffness sx in x-axis
        pos: position x,y,z in /torso_lift_link frame
        orientation: ox,oy,oz,ow in /torso_lift_link frame
        """

        # calc time to travel based on spatial and rotational velocity
        dt_translate = la.norm(np.array(pos) - np.array(self.last_pos_cmd)) / self.vel

        # TODO fix math
        dt_rotate = la.norm(2 * np.arccos(np.dot(self.last_orientation_cmd, orientation))) / self.rot_vel

        # rospy.loginfo("dt_translate: {0}, dt_rotate: {1}".format(dt_translate, dt_rotate))

        dt = max(dt_translate, dt_rotate) + 0.1 # add 100 ms delay to all commands

        # add a goal
        self.add_stiff_goal(pos,
                            [sx, MAX_LIN_STIFFNESS, MAX_LIN_STIFFNESS],
                            orientation,
                            self.t + dt)

        # advance state
        self.last_pos_cmd = pos
        self.last_orientation_cmd = orientation
        self.t += dt

    def add_loiter_goal(self, time):
        """add a goal to stay at the last command for a certain amount of time"""
        self.add_goal(self.last_pos_cmd,
                      [100, MAX_LIN_STIFFNESS, MAX_LIN_STIFFNESS],
                      self.last_orientation_cmd,
                      self.t + time)
        self.t += time

    def add_home_goal(self):
        """add goal to home near whiteboard"""
        self.add_move_goal(self.home_pos, [0,0,0,1], 50)

    def send(self):
        """wrap ee_cart_imped_action sendGoal"""
        self.control.sendGoal()

    def add_goal(self, pt, stiffness, orientation, t):
        return self.add_stiff_goal(pt, stiffness, orientation, t)

    def add_stiff_goal(self, pt, stiffness, orientation, t):
        """
        pt: set point x,y,z in /torso_lift_link frame
        stiffness: stiffness in x,y,z
        orientation: set orientation ox,oy,oz,ow in /torso_lift_link frame
        t: goal time
        """

        x, y, z = pt
        ox, oy, oz, ow = orientation
        sx, sy, sz = stiffness

        if t <= self.last_t:
            raise Exception('time ({0}) must be later than the last time ({1}) in the trajectory'.format(t, self.last_t))
        elif (sx > MAX_LIN_STIFFNESS) or (sy > MAX_LIN_STIFFNESS) or (sz > MAX_LIN_STIFFNESS):
            raise Exception('stiffness {0} too high!'.format(stiffness))
        else:
            self.last_t = t
            self.control.addTrajectoryPoint(x, y, z,
                                            ox, oy, oz, ow,
                                            sx, sy, sz,
                                            MAX_ROT_STIFFNESS, MAX_ROT_STIFFNESS, MAX_ROT_STIFFNESS,
                                            False, False, False,
                                            False, False, False,
                                            t,
                                            '/torso_lift_link')

    def add_force_goal(self, pt, stiffness, orientation, t):
        """
        pt: set point x,y,z in /torso_lift_link frame
        force: force/stiffness in x,y,z
        orientation: set orientation ox,oy,oz,ow in /torso_lift_link frame
        t: goal time
        """
        x, y, z = pt
        ox, oy, oz, ow = orientation
        fx, sy, sz = stiffness

        if t <= self.last_t:
            raise Exception('time ({0}) must be later than the last time ({1}) in the trajectory'.format(t, self.last_t))
        elif (sy > MAX_LIN_STIFFNESS) or (sz > MAX_LIN_STIFFNESS):
            raise Exception('stiffness {0} too high!'.format(stiffness))
        elif (fx > MAX_LIN_FORCE):
            raise Exception('force {0} too high!'.format(fx))
        else:
            self.last_t = t
            self.control.addTrajectoryPoint(x, y, z,
                                            ox, oy, oz, ow,
                                            fx, sy, sz,
                                            MAX_ROT_STIFFNESS, MAX_ROT_STIFFNESS, MAX_ROT_STIFFNESS,
                                            True, False, False,
                                            False, False, False,
                                            t,
                                            '/torso_lift_link')
