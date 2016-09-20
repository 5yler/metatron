#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
//Adapted From http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals

#include <yaml-cpp/yaml.h>
#include <tf/transform_datatypes.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

std::string plan_frame_id;
double waypoint_timeout;
std::string plan_file;
std::string plan_name;

std::vector<geometry_msgs::PoseStamped> goals;
int plan_length;
int failed = 0;

bool loop;


bool LoadPlan(std::string file_name)
{
  ROS_INFO("Loading nav plan file %s", file_name.c_str());

  YAML::Node plan = YAML::LoadFile(file_name);

 //$ dump contents to debug
  std::cout << YAML::Dump(plan) << std::endl;

  if (plan["name"])
  {
    plan_name = plan["name"].as<std::string>();
    ROS_WARN("Loading plan %s", plan_name.c_str());
  }

  if (plan["goals"])
  {
    plan_length = static_cast<int>(plan["goals"].size());

    for(unsigned int i = 0; i < plan_length; i++)
    {
      geometry_msgs::PoseStamped waypoint;
      if(plan["goals"][i]["x"])
      {
        waypoint.pose.position.x = plan["goals"][i]["x"].as<double>();
      }
      else
      {
        ROS_ERROR(".yaml waypoint %d has no x value",i);
        return false;
      }
      if(plan["goals"][i]["y"])
      {
        waypoint.pose.position.y = plan["goals"][i]["y"].as<double>();
      }
      else
      {
        ROS_ERROR(".yaml waypoint %d has no y value",i);
        return false;
      }
      double yaw;
      if(plan["goals"][i]["yaw"])
      {
        yaw = plan["goals"][i]["yaw"].as<double>();
      }
      else
      {
        ROS_WARN(".yaml waypoint %d has no yaw value, using 0 yaw",i);
        yaw = 0;
      }

      waypoint.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      waypoint.header.frame_id = plan_frame_id; 
      waypoint.header.stamp = ros::Time::now();

      goals.push_back(waypoint);
      ROS_INFO("Added waypoint %d with x: %4.1f, y: %4.1f, yaw: %4.1f", i+1, waypoint.pose.position.x, waypoint.pose.position.y, yaw);

    }

    ROS_WARN("Done loading %d waypoints.", plan_length);
  }
  else
  {
    ROS_ERROR("No goals in .yaml plan file, exiting");
    return false;
  }

  if (plan["loop"]) {
    loop = plan["loop"].as<bool>();
  }
  else {
    loop = false;
    ROS_WARN("Plan .yaml file does not specify 'loop' parameter, defaulting to no looping");
  }
  return true;

}

int main(int argc, char** argv){
  ros::init(argc, argv, "nav_goal_publisher");

  //$ private node handle to get parameters
  ros::NodeHandle n("~");  

  n.param<std::string>("plan_frame_id", plan_frame_id, "map");
  n.param<std::string>("plan_file", plan_file, "ERROR");
  n.param("waypoint_timeout", waypoint_timeout, 60.0);

  //$ load plan from YAML file
  if (!LoadPlan(plan_file))
  {
    return false;
  }  

  while (n.ok()) 
  {
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_base action server");
    }

    for (int i = 0; i < goals.size(); i++)
    {
      
      move_base_msgs::MoveBaseGoal goal;
      //Define goal

      goal.target_pose = goals[i];
      ROS_INFO("Sending goal %d.", i+1);
      ac.sendGoal(goal);

      ac.waitForResult(ros::Duration(waypoint_timeout));

      actionlib::SimpleClientGoalState goal_state = ac.getState();

      if(goal_state == actionlib::SimpleClientGoalState::SUCCEEDED) 
      {
        ROS_INFO("Goal reached!");

        //$ reset counter to zero
        if (i == goals.size()-1) 
        {
          if (loop == true) 
          {
            i = 0;
            ROS_WARN("Looping enabled, resetting to first goal point in plan.");

          } 
          else 
          {
            ROS_WARN("Plan executed successfully, all goals reached. Exiting...");
            ros::shutdown(); //idk if this is good practice
          }
        }
      } 
      else 
      {
        std::string state_string = goal_state.toString();
        ROS_ERROR("Goal state %s after %4.1f seconds", state_string.c_str(), waypoint_timeout);
        failed++;
        if (failed > plan_length)
        {
          ROS_ERROR("Failed to reach any of %d points in plan. I give up.", plan_length);
          ros::shutdown();
        }
      }
    }
  }

  return 0;
}