#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_draw')
import rospy

import draw_control as dc

if __name__ == '__main__':
    rospy.init_node('r_setup')

    draw_control = dc.DrawController('right_arm')

    # move to center of body
    # draw_control.add_goal((0.5,0,0), (1000, 1000, 1000), (0,0,0,1), 10)

    draw_control.move([0.5, 0, 0], [0,0,0,1], 1000)

    # move ahead slightly and tilt hand
    draw_control.home()

    draw_control.sendGoal()
