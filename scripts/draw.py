#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_draw')
import rospy
import tf
# from tf.transformations import quaternion_from_euler as quaternion
import numpy as np
import draw_control as dc

from scipy import interpolate

def pt(x0,x1):
    return np.array([x0, x1])

class DrawingPlan(object):
    """creates a trajectory to draw a shape"""
    def __init__(self, draw_control):
        super(DrawingTrajectory, self).__init__()
        self.draw_control = draw_control
        self.whiteboard = 0.8 # whiteboard defined as plane in at x-offset

    def send(self, path):
        # path defined as sequence of (x,y) vertices on whiteboard

        # orientation = self.draw_control.home_orientation
        strx = 100 # stiffness along x
        angle = -np.pi / 6

        for i in xrange(len(path) - 1):
            p1 = path[i+1]
            p0 = path[i]

            # method 1: roll wrist and then pitch to drawing angle
            # roll = np.atan2((p1[1] - p0[1]) / (p1[0] - p0[0]))
            # orientation = tf.transformations.quaternion_from_euler(roll, pitch, 0)

            # method 2: construct quaternion to rotate arbitrarily about axis perpendicular to current path
            perp = np.cross([1,0,0], p1 - p0)
            orientation = tf.transformations.quaternion_about_axis(angle, perp)

            self.draw_control.move(p0, orientation, sx)

    def interpolate(self, path):
        step = 0.01 # 1 cm
        # interp_x = []
        # interp_y = []

        interp_path = []

        for i in xrange(len(path) - 1):
            p0 = path[i]
            p1 = path[i+1]
            interp_path.append(p0)
            distance = np.norm(p1 - p0)
            direction = (p1 - p0) / distance
            new_p = p0
            while True:
                new_p += step * direction
                if np.norm(new_p - p0) <= distance:
                    interp_path.append(new_p)
                else:
                    break
            # interp_path.append(p1) # not necessary

        retrun interp_path


    def square(self, length):
        # build trajectory that draws a square
        start_pos = self.draw_control.curr_pos()
        rospy.loginfo("Drawing square starting at {0}".format(start_pos))

        # use y,z as coordinates
        start = pt(start_pos[1], start_pos[2])

        path = []
        path.append(start + pt(0,0))
        path.append(start + pt(0,length))
        path.append(start + pt(length,0))
        path.append(start + pt(length,length))
        path.append(start + pt(0,length))
        path.append(start + pt(0,0))
        path.append(start + pt(0,0))

        path_interp = self.interpolate(path)

        self.send(path_interp)

if __name__ == '__main__':
    rospy.init_node('draw')

    draw_control = dc.DrawController('right_arm', True) # stiffness
    drawing = DrawingPlan(draw_control)

    draw_control.home()
    drawing.square()
    draw_control.home()

    draw_control.sendGoal()
