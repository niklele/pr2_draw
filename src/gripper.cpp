#include "gripper.h"

Gripper::Gripper() {
  //Initialize the client for the Action interface to the gripper controller
  //and tell the action client that we want to spin a thread by default
  m_gripperClient = new GripperClient("r_gripper_controller/gripper_action", true);

  //wait for the gripper action server to come up
  while(!m_gripperClient->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the r_gripper_controller/gripper_action action server to come up");
  }
}

Gripper::~Gripper() {
  delete m_gripperClient;
}

void Gripper::move(float position, float maxEffort) {
  pr2_controllers_msgs::Pr2GripperCommandGoal cmd;
  cmd.command.position = position;
  cmd.command.max_effort = maxEffort;

  ROS_INFO_STREAM("Sending move goal: " << position << " with max effort: " << maxEffort);
  m_gripperClient->sendGoal(cmd);
  m_gripperClient->waitForResult();
  if(m_gripperClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO_STREAM("Gripper move success: " << position << " with max effort: " << maxEffort);
  } else {
    ROS_INFO_STREAM("Gripper move fail: " << position << " with max effort: " << maxEffort);
  }
}

void Gripper::open() {
  move(0.08, -1.0); // Do not limit effort (negative)
}

void Gripper::close() {
  move(0.0, 50.0); // Close gently
}

int main(int argc, char** argv){
  ros::init(argc, argv, "gripper");
  Gripper gripper;

  gripper.open();
  ros::Duration(5.0).sleep(); // sleep for 5 seconds
  gripper.close();

  return 0;
}
