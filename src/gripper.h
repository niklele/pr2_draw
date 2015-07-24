#ifndef GRIPPER_H
#define GRIPPER_H

#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>

/**
 * Adapted from http://wiki.ros.org/pr2_controllers/Tutorials/Moving%20the%20gripper
 */

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

class Gripper{

private:
  GripperClient *m_gripperClient;

  // Move the Gripper to the desired position given an amount of effort
  void move(float position, float maxEffort);

public:
  Gripper();

  ~Gripper();

  // Open the gripper completely
  void open();

  // Close the gripper completely
  void close();

};

#endif /* GRIPPER_H */
