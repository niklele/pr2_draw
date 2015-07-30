#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_draw')
import rospy
import tf
import numpy as np
import numpy.linalg as la
import draw_control as dc

def pt(x, y, z):
    return np.array([x, y, z])

def quat(x, y, z, w):
    return np.array([x, y, z, w])

def calc_orientation(p0, p1, angle):
    """
    Calculate the gripper orientation required to stay at a certain angle to the segment from p0 to p1
    p0 and p1 should be in /r_gripper_tool_frame
    """
    # method 1: roll wrist and then pitch to drawing angle
    # roll = np.atan2((p1[1] - p0[1]) / (p1[0] - p0[0]))
    # orientation = tf.transformations.quaternion_from_euler(roll, pitch, 0)

    # method 2: construct quaternion to rotate arbitrarily about axis perpendicular to current segment
    perp = np.cross([1,0,0], p1 - p0)
    orientation = tf.transformations.quaternion_about_axis(angle, perp)
    return orientation

def calc_path_orientations(path):
    """change path's orientations so that the marker stays at a certain angle to the drawing"""
    # path is list of [pt, quat]
    angle = -np.pi/6 # fixed angle
    for i in xrange(len(path) - 1):
        p0 = path[i][0]
        p1 = path[i+1][0]
        path[i][1] = calc_orientation(p0, p1, angle)

    try:
        path[-1][1] = path[-2][1] # use 2nd to last orientation for last orientation
    except IndexError:
        pass
    return path

def interpolate(path, max_dist):
    """
    Interpolate a path along position and orientation waypoints.
    Ensures that each point is at most max_dist away from the next.
    """

    # 1. linear interpolate positions with max_dist
    # 2. slerp orientations to match interpolated positions

    interp_path = []
    for i in xrange(len(path) - 1):
        p0 = path[i][0]
        p1 = path[i+1][0]

        interp_pos = [] # (pos, fractional distance along segment)

        interp_pos.append((p0, 0.0)) # first point

        distance = la.norm(p1 - p0)
        direction = (p1 - p0) / distance
        new_p = np.array(p0, copy=True)

        rospy.loginfo("interp step {0} of {1}".format(i+1, len(path)))
        rospy.loginfo("position interpolation")

        while True:
            new_p += step * direction
            diff = la.norm(new_p - p0, 2)
            if diff <= distance:
                interp_pos.append((new_p, diff/distance))
            else:
                break

        interp_path.append((p1, 1.0)) # last point

        rospy.loginfo("orientation interpolation")
        q0 = path[i][1]
        q1 = path[i+1][1]

        for pos, fraction in interp_pos:
            new_q = tf.transformations.quaternion_slerp(q0, q1, fraction)
            interp_path.append([pos, new_q])

    return interp_path

def make_square(start_pos, length):
    """create position waypoints to follow a square"""
    path = []
    path.append([start + pt(0, 0, 0), quat(0, 0, 0, 1)])
    path.append([start + pt(0, 0, length), quat(0, 0, 0, 1)])
    path.append([start + pt(0, length, 0), quat(0, 0, 0, 1)])
    path.append([start + pt(0, length, length), quat(0, 0, 0, 1)])
    path.append([start + pt(0, 0, length), quat(0, 0, 0, 1)])
    path.append([start + pt(0, 0, 0), quat(0, 0, 0, 1)])
    path.append([start + pt(0, 0, 0), quat(0, 0, 0, 1)])
    return path

def make_line(start_pos, length):
    """create position waypoints to follow a line"""
    path = []
    path.append([start + pt(0, 0, 0), quat(0, 0, 0, 1)])
    # path.append([start + pt(0, length/2, 0), quat(0, 0, 0, 1)])
    path.append([start + pt(0, length, 0), quat(0, 0, 0, 1)])
    return path

if __name__ == '__main__':
    rospy.init_node('draw')
    draw_control = dc.DrawController()

    start_offset = np.array([0.01, 0, 0]) # offset in /r_gripper_tool_frame

    # 1. create position waypoints
    # 2. create orientation waypoints to match positions
    # 3. interpolate path

    start_pos = []
    while not start_pos:
        start_pos = draw_control.curr_pos + start_offset
    rospy.loginfo("Calculating line starting at {0}".format(start_pos))

    line = make_line(start_pos, 0.2)
    line_orientations = calc_path_orientations(line)
    line_orientations_interp = interpolate(line_orientations, 0.01) # 1 cm max step

    # Send the path
    draw_control.add_home_goal()
    draw_control.add_path_goals(line_orientations_interp, 100)
    draw_control.add_home_goal()
    rospy.loginfo("Sending Goal Trajectory")
    draw_control.send()

    rospy.spin()
