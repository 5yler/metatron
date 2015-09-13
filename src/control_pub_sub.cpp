 /**
 * control_pub_sub.cpp
 * Gigatron publisher for steering and motor control using rosserial and Arduino
 * 
 * @author  Syler Wagner  <syler@mit.edu>
 * @date    2015-09-09    creation
 *
 * This node listens for Twist messages with forward and angular velocity values,
 * applies the dynamic model of the car, and publishes ROS msgs to a sketch on the Arduino
 * in order to control the steering and motors.
 * 
 * Usage Instructions:
 * 1. Startup your roscore and the rosserial python node in their own terminal windows.
 *      roscore
 *      rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0
 * 2. Start control_pub_sub in a separate window to send controls
 *      rosrun gigatron control_pub_sub
 * 3. Start whatever node sends geometry_msgs/Twist messages. For testing, you can use 
 *      rosrun turtlesim turtle_teleop_key /turtle1/cmd_vel:=cmd_vel 
 **/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"

   // const int ZERO_STEERING_ANGLE_PWM = 128;
    const double STEERING_PWM_RANGE = 255.0;

    const double PI = 3.141592653589793238463;
    const double ZERO_STEERING_ANGLE = 0.0; // [radians]
    const double STEERING_ANGLE_RANGE = 50 * (PI / 180); //$ [radians] this is the correct steering range
    const double ABS_MAX_STEERING_ANGLE = 25 * (PI / 180); //$ [radians]


class ControlPubSub
{
public:
  ControlPubSub()
  {
    sub_ = n_.subscribe("cmd_vel", 1000, &ControlPubSub::velCommandCallback, this);
    pub_ = n_.advertise<geometry_msgs::Vector3>("control", 1000);

    // odometry stuff - move eventually
    osub_ = n_.subscribe("odo_val", 1000, &ControlPubSub::odoCallback, this);
    cmdsub_ = n_.subscribe("command", 1000, &ControlPubSub::cmdCallback, this);
  }

  void cmdCallback(const geometry_msgs::Vector3::ConstPtr& cmdmsg) 
  {

    unsigned int servoPWM = (unsigned int) cmdmsg->x;
    unsigned int leftPWM  = (unsigned int) cmdmsg->y;
    unsigned int rightPWM = (unsigned int) cmdmsg->z;

    ROS_INFO_STREAM("/command: \tSPWM\t" << servoPWM << "\tLRPM\t" << leftPWM << "\tRRPM\t" << rightPWM);

  }

  void odoCallback(const geometry_msgs::Vector3::ConstPtr& odomsg) 
  {

    double servoPWM = odomsg->x;
    double leftRPM  = odomsg->y;
    double rightRPM = odomsg->z;

    ROS_INFO_STREAM("/odo_val \tSPWM\t" << servoPWM << "\tLRPM\t" << leftRPM << "\tRRPM\t" << rightRPM);

    double steeringAngle = STEERING_ANGLE_RANGE * (servoPWM / STEERING_PWM_RANGE) - ABS_MAX_STEERING_ANGLE;
  
  }

  void velCommandCallback(const geometry_msgs::Twist::ConstPtr& msg) 
  {
    double desiredForwardVelocity = msg->linear.x;   // desired forward velocity 
                                              // ignore linear.y since car can't go sideways
    double desiredAngularVelocity = msg->angular.z;  // angular velocity

    // define what we're looking for
    double desiredSteeringAngle;
    double desiredRightWheelVelocity;
    double desiredLeftWheelVelocity;

    double carLength  = 0.5842; // [m] center-to-center distance between front and back wheels
    double carWidth   = 0.7239; // [m] center-to-center between right and left wheels

    // calculate desired steering angle and wheel velocities
    desiredSteeringAngle = std::atan2(carWidth * desiredAngularVelocity, desiredForwardVelocity);
    desiredLeftWheelVelocity  = desiredForwardVelocity - (desiredAngularVelocity / carWidth);
    desiredRightWheelVelocity = desiredForwardVelocity + (desiredAngularVelocity / carWidth);

    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    geometry_msgs::Vector3 control;

    // add steering angle and desired velocities to message
    control.x = desiredSteeringAngle; // TODO: CONVERT
    control.y = desiredLeftWheelVelocity;
    control.z = desiredRightWheelVelocity;
    
    // ROS_INFO_STREAM is a replacement for cout
    ROS_INFO_STREAM("/control: \tTheta\t" << control.x << "\tLeftV\t" << control.y << "\tRightV\t" << control.z);

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    pub_.publish(control);

  }
private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::Subscriber osub_;
  ros::Subscriber cmdsub_;

}; // end of class ControlPubSub

/**
 * This node demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "control_pub_sub");

  // create a ControlPubSub object to publish and subscribe at same time
  ControlPubSub cps;

  ros::spin();

  return 0;
}

