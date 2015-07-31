# pr2_draw

Simple ROS package that allows the PR2 to draw on a whiteboard. Depends on [ee_cart_imped](http://wiki.ros.org/ee_cart_imped) for force/stiffness arm control.

## Instructions
1. download [ee_cart_imped](http://wiki.ros.org/ee_cart_imped)
2. start PR2
3. `rosrun pr2_tuckarm tuck_arms.py -r t -l t`
  - start the process with arms tucked in
4. `roslaunch ee_cart_imped_action r_arm_cart_imped_action.launch`
  - starts `ee_cart_imped` action server for right arm only
  - NOTE: other right arm control won't work while this is running
5. on PR2: `roslaunch haptics haptics_tf_echo.launch`
  - Provides a `tf_echo` node that broadcasts the transform from `/torso_lift_link` to `/r_gripper_tool_frame` so that `pr2_draw` doesn't need to deal with the TF directly.
6. `rosrun pr2_draw setup.py`
  - moves the right arm into a ready position
7. `rosrun pr2_draw gripper`
  - opens the gripper for 5 seconds and then closes it
  - place the marker while gripper is open
8. `rosrun pr2_draw draw.py`
  - plans smooth trajectory to trace out a plan made of position waypoints
  - tested on simple shapes like squares
