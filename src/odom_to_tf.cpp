#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"

class OdometryComputer {
public:

  unsigned int leftWheelRPM = 0;
  unsigned int rightWheelRPM = 0;
  double x = 0.0;       //m/s
  double y = 0.0;       //m/s
  double theta = 0.0;   //rad, where 0 is forward
  ros::Time lastTime = ros::Time::now();
  ros::Time currentTime = ros::Time::now();
  double wheelRadius = 0.5; //TODO actual value

  void rpmCallback(geometry_msgs::Vector3::ConstPtr& rpm) {
    leftWheelRPM = rpm->y;
    rightWheelRPM = rpm->z;
  }

private:

  ros::NodeHandle n;
  tf::TransformBroadcaster odom_broadcaster;
  ros::Publisher odom_pub;
  ros::Subscriber rpm_sub;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "odometry_publisher");

  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  rpm_sub = n.subscribe("odo_val", 1000, OdometryComputer::rpmCallback);

  ros::Rate r(1000.0);

  while(n.ok()) {
    currentTime = ros::Time::now();
    double rightWheelVelocity = double(rightWheelRPM) * wheelRadius * 0.10472;
    double leftWheelVelocity = double(leftWheelRPM) * wheelRadius * 0.10472;

    double dt = (currentTime - lastTime).toSec();

    double wheelBaseWidth = 0.5;

    double centerVelocity = (rightWheelVelocity + leftWheelVelocity) / 2;

    x += centerVelocity * dt * cos(theta);
    y += centerVelocity * dt * sin(theta);
    theta += (rightWheelVelocity - leftWheelVelocity) * dt / wheelBaseWidth;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

    geometry_messages::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.translation.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
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
