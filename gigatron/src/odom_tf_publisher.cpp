/********************************************************************
  Software License Agreement (BSD License)

  Copyright (c) 2017, Cult Classic Racing.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

  1. Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above
     copyright notice, this list of conditions and the following
     disclaimer in the documentation and/or other materials provided
     with the distribution.
  3. Neither the name of the copyright holder nor the names of its 
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************/

 /**
 * odom_tf_publisher.cpp
 * Gigatron publisher for odometry and odometry transform.
 * 
 * @author  Chris Desnoyers   <cjdesno@mit.edu>
 * @author  Syler Wagner      <syler@mit.edu>

 * @date    2015-09-12  cjdesno   creation
 * @date    2015-09-13  syler     added proper model dimensions 
 * @date    2016-08-10  syler     adapted to work with new custom messages
 * @date    2016-08-17  syler     refactored to take in parameters instead of hardcoding
 *
 * This node listens for odometry sensor readings from the Arduino, 
 * then publishes Odometry messages and broadcasts associated transforms.
 **/

#include "ros/ros.h"
#include <gigatron_msgs/ExtendedState.h>
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"

#define PI 3.141592653589793238463

double left_wheel_velocity_ = 0.0;
double right_wheel_velocity_ = 0.0;

std::string odom_topic_;
std::string state_topic_;
int rate_;

double _wheelbase_width;

void driveStateCallback(const gigatron_msgs::ExtendedState::ConstPtr& msg)
{
  left_wheel_velocity_ = msg->drive.vel_left;
  right_wheel_velocity_ = msg->drive.vel_right;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "odom_tf_publisher");

  ros::NodeHandle n("~");
  tf::TransformBroadcaster odom_broadcaster;

  //$ load parameters
  n.param("rate", rate_, 200); //$ default rate 200 Hz
  n.param<std::string>("odom_topic", odom_topic_, "odom");
  n.param<std::string>("state_topic", state_topic_, "state");

  std::string name;
  ros::param::param<std::string>("/car/name", name, "ERROR");
  ROS_ERROR("Loading parameters for %s frame...", name.c_str());

  if (ros::param::has("/car/wheelbase_width"))
  {
    ros::param::get("/car/wheelbase_width", _wheelbase_width);
    ROS_WARN("Wheelbase width: %4.4f", _wheelbase_width);

  }
  else
  {
    ROS_ERROR("No wheelbase_width parameter set. Make sure your config files are correct!");
    ROS_WARN("Exiting...");
    ros::shutdown();
  }


  //$ initialize publishers and subscribers
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>(odom_topic_, 5);
  ros::Subscriber rpm_sub = n.subscribe(state_topic_, 10, driveStateCallback);
  ros::Rate r(rate_);

  ROS_WARN("Subscribed to car state topic: %s", state_topic_.c_str());
  ROS_WARN("Publishing to odometry topic: %s at %d Hz", odom_topic_.c_str(), rate_);

  double x = 0.0;       // [m/s]
  double y = 0.0;       // [m/s]
  double theta = 0.0;   // [rad] yaw angle, where 0 is forward
  ros::Time last_time = ros::Time::now();
  ros::Time current_time = ros::Time::now();

  while(n.ok()) {
    ros::spinOnce();
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();

    double center_velocity = (right_wheel_velocity_ + left_wheel_velocity_) / 2;

    x += center_velocity * dt * cos(theta);
    y += center_velocity * dt * sin(theta);
    theta += (right_wheel_velocity_ - left_wheel_velocity_) * dt / _wheelbase_width;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //TODO: For the robot_localization ekf nodes
    //odom.pose.covariance[35] = ???;

    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = center_velocity;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = (right_wheel_velocity_ - left_wheel_velocity_) / _wheelbase_width;

    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}
