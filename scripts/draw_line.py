#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_draw')
import rospy

import draw_control as dc

if __name__ == '__main__':
    rospy.init_node('draw_line')

    draw_control = dc.DrawController('right_arm')

    orientation = draw_control.home_orientation

    """WARNING: use setup first!"""
    draw_control.home()

    draw_control.move((0.79,0,0), orientation, 200)
    draw_control.move((0.79,0,-0.1), orientation, 100)
    draw_control.move((0.75,0,-0.1), orientation, 100)

    draw_control.home()

    draw_control.sendGoal()
