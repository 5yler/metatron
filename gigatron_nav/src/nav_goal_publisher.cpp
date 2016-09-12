#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>

//Adapted From http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

geometry_msgs::PoseStamped goals [3]; //this won't work. Need to type correctly
ros::param::get("goals",goals);
int goal_counter = 0;

int main(int argc, char** argv){
  ros::init(argc, argv, "nav_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server");
  }

  move_base_msgs::MoveBaseGoal goal;
  //Define goal
  goal.target_pose.header.frame_id = "world"; //probably want goals in world frame right? map frame?
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = goals[goal_counter][0];
  goal.target_pose.pose.position.y = goals[goal_counter][1];

  /*
  //gotta put theta somewhere
  goal.target_pose.pose.orientation.x = 0.0; //do we care how the robot is facing at its goal? Probably so its not backwards
  goal.target_pose.pose.orientation.y = 0.0; //how to convert to this from theta?
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 0.0;
  */

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Goal Reached!");
    goal_counter++;
    if (goal_counter == 3) {
      if (loop == true) {
        goal_counter = 0;
      } else {
        ros::shutdown();
      }
    }
  } else {
    ROS_WARN("Issue reaching goal.");
  }

  return 0;
}