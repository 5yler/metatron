//placeholder
int main() {
  ros::NodeHandle n;
  tf::TransformBroadcaster odom_broadcaster;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

  float theta = 0;
  float x = 0;
  float y = 0;

  float rightWheelVelocity = 0;
  float leftWheelVelocity = 0;
  float steeringAngle = 0;

  float lastTime = 1;
  float thisTime = 2;

  float wheelBaseWidth = 0.5;

  float centerVelocity = (rightWheelVelocity + leftWheelVelocity) / 2;
  float dt = thisTime - lastTime;

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
}
