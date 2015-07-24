#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_draw')
import rospy

import stiffness_control as sc

if __name__ == '__main__':
    rospy.init_node('draw_line')

    arm_control = sc.StiffnessController('right_arm')

    """WARNING: use r_setup first!"""
    arm_control.home()

    arm_control.move((0.79,0,0), 200)
    arm_control.move((0.79,0,-0.1), 100)
    arm_control.move((0.75,0,-0.1), 100)

    arm_control.home()

    arm_control.sendGoal()
