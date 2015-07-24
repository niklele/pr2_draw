#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_draw')
import rospy

import stiffness_control as sc

if __name__ == '__main__':
    rospy.init_node('r_setup')

    arm_control = sc.StiffnessController('right_arm')

    # move to center of body
    arm_control.add_stiff_goal((0.5,0,0), (1000, 1000, 1000), (0,0,0,1), 10)

    # move ahead slightly and tilt hand
    arm_control.home()

    arm_control.sendGoal()
