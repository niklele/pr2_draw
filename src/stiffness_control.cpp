#include <ee_cart_imped_action/ee_cart_imped_arm.hh>
#include <ee_cart_imped_msgs/EECartImpedGoal.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "stiffness_control_cpp");

  /**
   *The EECartImpedArm class is a wrapper for an action client to the
   *ee_cart_imped_action server.  The argument "r_arm_cart_imped_controller"
   *tells the client that it is a client for the server that controls the
   *right arm
   */
  EECartImpedArm arm("r_arm_cart_imped_controller");

  /**
   *This variable will hold the trajectory as we create it.
   */
  ee_cart_imped_msgs::EECartImpedGoal traj;

  /**
   *addTrajectoryPoint is a static function in the EECartImpedArm class that
   *adds a trajectory point to the end of the first argument.  It simply
   *assigns each value in the goal structure for us to prevent having to
   *write it all out.
   *
   *This is a point in the center of the robot's body.  This simply moves the
   *arm to that point with maximum stiffness.
   */
  EECartImpedArm::addTrajectoryPoint(traj, 0.5, 0, 0, 0, 0, 0, 1,
                                     1000, 1000, 1000, 30, 30, 30,
                                     false, false, false, false, false,
                                     false, 4, "/torso_lift_link");
  /**
   *This point is farther in front of the robot, but it is only allowed to
   *use a very small stiffness in the x direction
   */
  EECartImpedArm::addTrajectoryPoint(traj, 0.75, 0, 0, 0, 0, 0, 1,
                                     50, 1000, 1000, 30, 30, 30,
                                     false, false, false, false, false,
                                     false, 6, "/torso_lift_link");
  /**
   *This is the line that actually sends the trajectory to the action server
   *and starts the arm moving.  The server will block until the arm completes
   *the trajectory or it is aborted.
   */
  arm.startTrajectory(traj);
}
