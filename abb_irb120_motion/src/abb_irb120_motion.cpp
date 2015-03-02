#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "abb_irb120_motion");
  moveit::planning_interface::MoveGroup group("manipulator");

  // start a background "spinner", so our node can process ROS messages
  //  - this lets us know when the move is completed
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::map<std::string, double> home;
  home["joint_1"] =  0.00;
  home["joint_2"] =  0.00;
  home["joint_3"] =  0.00;
  home["joint_4"] =  0.00;
  home["joint_5"] =  0.00;
  home["joint_6"] =  0.00;

  //move to pre-defined position
  group.setNamedTarget("all_zero");
  group.move();

  //Create a cartesian position using the desired (x,y,z) tool position
  //and rotate so that target orientation is facing towards the table
  Eigen::Affine3d approach = Eigen::Translation3d(0.466, 0.000, 0.246)
                         * Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY());
  group.setPoseTarget(approach);
  group.move();

  //Offset the X coordinate by a small amount to accomplish the "pick" motion
  Eigen::Affine3d pick = approach.translate(0.2*Eigen::Vector3d::UnitX());
  group.setPoseTarget(pick);
  group.move();

  //Move to the retreat position by offsetting the X coordinate (same as approach in this instance)
  Eigen::Affine3d retreat = pick.translate(-0.2*Eigen::Vector3d::UnitX());
  group.setPoseTarget(retreat);
  group.move();

  //Create a std::vector<double> containing the desired joint positions of the Inspect Position
  std::vector<double> inspectPos;
  inspectPos.push_back(-0.5); inspectPos.push_back(1.07); inspectPos.push_back(-0.52);
  inspectPos.push_back(-0.0); inspectPos.push_back(1.01); inspectPos.push_back(-0.5);
  group.setJointValueTarget(inspectPos);
  group.move();

  //Return to the home position, as defined by the joint angles given at the beginning
  group.setJointValueTarget(home);
  group.move();




  //Additional commands not used above

  // Eigen::Affine3d pick = Eigen::Translation3d(0.466, 0.000, 0.06)
  //                        * Eigen::Quaterniond(0.706, 0.000, 0.709, -0.000);
  // group.setPoseTarget(pick);
  // group.move();

  // group.setRandomTarget();
  // group.move();
}
