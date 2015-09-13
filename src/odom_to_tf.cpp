 /**
 * odom_to_tf.cpp
 * Gigatron publisher for odometry and odometry transform.
 * 
 * @author  Chris Desnoyers   <cjdesno@mit.edu>
 * @author  Syler Wagner      <syler@mit.edu>

 * @date    2015-09-12  cjdesno   creation
 * @date    2015-09-13  syler     added proper model dimensions 
 *
 * This node listens for odometry sensor readings from the Arduino, 
 * then publishes Odometry messages and broadcasts associated transforms.
 **/

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"

const static double INCHES_TO_M = 0.0254; //$ conversion from inches to meters
const static double PI = 3.141592653589793238463;

const static double wheelBaseWidth = 23.0 * INCHES_TO_M;  //$ [m]
const static double wheelRadius = 4.90 * INCHES_TO_M;     //$ [m]
const static double gearRatio = 11.0 / 60.0;  //$ gear ratio between motor and wheels

double leftWheelVelocity = 0.0;
double rightWheelVelocity = 0.0;

void velocityCallback(geometry_msgs::Vector3::ConstPtr& vel) {
  leftWheelVelocity = vel->y;
  rightWheelVelocity = vel->z;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  tf::TransformBroadcaster odom_broadcaster;

  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Subscriber rpm_sub = n.subscribe("odo_val", 1000, velocityCallback);

  ros::Rate r(1000.0);

  double x = 0.0;       // [m/s]
  double y = 0.0;       // [m/s]
  double theta = 0.0;   // [rad] where 0 is forward
  ros::Time lastTime = ros::Time::now();
  ros::Time currentTime = ros::Time::now();

  while(n.ok()) {
    currentTime = ros::Time::now();
    double dt = (currentTime - lastTime).toSec();

    double centerVelocity = (rightWheelVelocity + leftWheelVelocity) / 2;

    x += centerVelocity * dt * cos(theta);
    y += centerVelocity * dt * sin(theta);
    theta += (rightWheelVelocity - leftWheelVelocity) * dt / wheelBaseWidth;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = currentTime;
    odom_trans.header.frame_id = "odom";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = currentTime;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = centerVelocity;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = (rightWheelVelocity - leftWheelVelocity) / wheelBaseWidth;

    odom_pub.publish(odom);

    lastTime = currentTime;
    r.sleep();
  }
}
