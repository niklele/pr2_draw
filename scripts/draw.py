#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_draw')
import rospy
import tf
# from tf.transformations import quaternion_from_euler as quaternion
import numpy as np
import numpy.linalg as la
import draw_control as dc

from scipy import interpolate

def pt(x0,x1):
    return np.array([x0, x1])

class DrawingPlan(object):
    """creates a trajectory to draw a shape"""
    def __init__(self, draw_control):
        super(DrawingPlan, self).__init__()
        self.draw_control = draw_control
        self.whiteboard = 0.8 # whiteboard defined as plane in at x-offset

    def send(self, path):
        # path defined as sequence of (x,y) vertices on whiteboard

        # orientation = self.draw_control.home_orientation
        sx = 100 # stiffness along x
        angle = -np.pi / 6

        for i in xrange(len(path) - 1):
            p0 = np.array([self.whiteboard, path[i][0], path[i][1]])
            p1 = np.array([self.whiteboard, path[i+1][0], path[i+1][1]])

            # method 1: roll wrist and then pitch to drawing angle
            # roll = np.atan2((p1[1] - p0[1]) / (p1[0] - p0[0]))
            # orientation = tf.transformations.quaternion_from_euler(roll, pitch, 0)

            # method 2: construct quaternion to rotate arbitrarily about axis perpendicular to current path
            perp = np.cross([1,0,0], p1 - p0)
            orientation = tf.transformations.quaternion_about_axis(angle, perp)

            # actual = [self.whiteboard, p0[0], p0[1]]
            rospy.loginfo("Draw point: {0} orientation: {1}".format(p0, orientation))
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
            distance = la.norm(p1 - p0)
            direction = (p1 - p0) / distance
            new_p = np.array(p0, copy=True)

            rospy.loginfo("interp step {0} of {1}".format(i+1, len(path)))

            while True:
                new_p += step * direction

                diff = la.norm(new_p - p0, 2)

                # rospy.loginfo("p0:{0}, new_p:{1}, diff:{2}, dir:{3}".format(p0, new_p, diff, direction))

                if diff <= distance:
                    interp_path.append(new_p)
                else:
                    break
            # interp_path.append(p1) # not necessary


        return interp_path


    def square(self, length):
        # build trajectory that draws a square
        start_pos = []
        while not start_pos:
            start_pos = draw_control.curr_pos
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

    def line(self, length):

        start_pos = []
        while not start_pos:
            start_pos = draw_control.curr_pos
        rospy.loginfo("Drawing line starting at {0}".format(start_pos))

        # use y,z as coordinates
        start = pt(start_pos[1], start_pos[2])

        path = []
        path.append(start + pt(0, 0))
        path.append(start + pt(length/2, 0))
        path.append(start + pt(length, 0))

        rospy.loginfo("path: {0}".format(path))

        # path_interp = self.interpolate(path)
        # self.send(path_interp)

        self.send(path)

if __name__ == '__main__':
    rospy.init_node('draw')

    draw_control = dc.DrawController('right_arm', True) # stiffness
    drawing = DrawingPlan(draw_control)

    draw_control.home()
    drawing.square(0.1)

    # drawing.line(0.2)

    # draw_control.loiter(2) # seconds

    draw_control.home()

    draw_control.sendGoal()

    rospy.spin()
