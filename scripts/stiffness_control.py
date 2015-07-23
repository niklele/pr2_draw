#!/usr/bin/env python
import roslib; roslib.load_manifest('ee_cart_imped_tutorial')
import rospy
import ee_cart_imped_action

def main():
    control = ee_cart_imped_action.EECartImpedClient('left_arm')
    control.addTrajectoryPoint(0.5, 0, 0, 0, 0, 0, 1,
                               1000, 1000, 1000, 30, 30, 30,
                               False, False, False, False, False,
                               False, 4, '/torso_lift_link');
    control.addTrajectoryPoint(0.75, 0, 0, 0, 0, 0, 1,
                               50, 1000, 1000, 30, 30, 30,
                               False, False, False, False, False,
                               False, 6, '/torso_lift_link');
    control.sendGoal()

if __name__ == '__main__':
    rospy.init_node('stiffness_control_test')
    main()
