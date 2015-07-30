#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_draw')
import rospy

import draw_control as dc

if __name__ == '__main__':
    rospy.init_node('r_setup')

    draw_control = dc.DrawController()

    draw_control.add_move_goal([0.5, 0, 0], [0,0,0,1])

    # move ahead slightly and tilt hand
    draw_control.add_home_goal()

    draw_control.send()
