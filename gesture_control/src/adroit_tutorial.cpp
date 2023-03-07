#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_msgs/Float64.h>
#include "std_msgs/String.h"
#include <sstream>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle nh_;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string arm_pg = "arm";
  static const std::string pincer_pg = "pincer";

  moveit::planning_interface::MoveGroupInterface move_group_arm(arm_pg);
  moveit::planning_interface::MoveGroupInterface move_group_pincer(pincer_pg);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const robot_state::JointModelGroup* joint_model_group_arm =
    move_group_arm.getCurrentState()->getJointModelGroup(arm_pg);

  const robot_state::JointModelGroup* joint_model_group_pincer =
    move_group_pincer.getCurrentState()->getJointModelGroup(pincer_pg);

  ROS_INFO_NAMED("tutorial", "Arm Reference frame: %s", move_group_arm.getPlanningFrame().c_str());

  ROS_INFO_NAMED("tutorial", "Pincer Reference frame: %s", move_group_pincer.getPlanningFrame().c_str());



//   moveit::core::RobotStatePtr current_state_arm = move_group_arm.getCurrentState();

//   std::vector<double> joint_group_positions_arm;
//   current_state_arm->copyJointGroupPositions(joint_model_group_arm, joint_group_positions_arm);

//   moveit::planning_interface::MoveGroupInterface::Plan my_plan;

//   joint_group_positions_arm[0] = 1.0;
//  //   move_group_arm.setJointValueTarget(joint_group_positions_arm);

//   bool success = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//   ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");


  ros::Publisher pub_pos2 = nh_.advertise<std_msgs::Float64>("hdt_arm/pincer_joint_position_controller/command", 10);
//   ros::Publisher chatter_pub = nh_.advertise<std_msgs::Float64>("hdt_arm/joint1_position_controller/command", 10);
  std_msgs::Float64 command;
  std_msgs::Float64 add;
  add.data = 0.1;
  command.data = 0.0;
  ros::Rate loop_rate(10);
  
  std_msgs::String msg;
  std::stringstream ss;
  int count = 0;
  ss << "hello world " << count;
  msg.data = ss.str();
 //   move_group_arm.move(); 
  pub_pos2.publish(command);
//   chatter_pub.publish(command);
  count=0;
  int counter2=0;
  while (ros::ok())
  {
    // command.data +=add.data;
    loop_rate.sleep();
    command.data +=add.data;
    pub_pos2.publish(command);
    // count++;
    // ROS_INFO("COUNT  %d", count);

    // if (count>20)
    // {   
    //     counter2=1;
    //     command.data-=add.data;
    //     pub_pos2.publish(command);
    //     if (count < 40)
    //     {
    //         count=0;
    //         counter2=0;
    //     }
    // }
    // else if (counter2==0)
    // {
    //     command.data +=add.data;
    //     pub_pos2.publish(command);
    //     count++;
    //     ROS_INFO("COUNT  %d", count);
    // }
  }
  return 0;
};



