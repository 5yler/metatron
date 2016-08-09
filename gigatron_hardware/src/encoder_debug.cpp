 /**
 * encoder_debug.cpp
 * Gigatron publisher/subscriber for Arduino motor control debug
 * 
 * @author  Syler Wagner  <syler@mit.edu>
 * @date    2016-04-10    creation
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
#include "geometry_msgs/Vector3.h"
#include <boost/thread/thread.hpp>

//$ debugging messages
#include <gigatron/Motors.h>

#define PI 3.141592653589793238463
#define INCHES_TO_M 0.0254 //$ conversion from inches to meters
#define S_LOOP_INTERVAL 100.0

const double STEERING_PWM_RANGE = 255.0;

const double ZERO_STEERING_ANGLE = 0.0; // [radians]
const double STEERING_ANGLE_RANGE = 50 * (PI / 180); //$ [radians] this is the correct steering range
const double ABS_MAX_STEERING_ANGLE = 25 * (PI / 180); //$ [radians]

const static double gearRatio = 11.0 / 60.0;  //$ gear ratio between motor and wheels
const static double wheelRadius = 4.90 * INCHES_TO_M;     //$ [m]
const static double RPM_TO_M_S = (2 * PI * wheelRadius) / 60.0;   //$ conversion from RPM to meters per second


class EncoderDebug
{
public:
  EncoderDebug()
  {
    control_pub_ = n_.advertise<geometry_msgs::Vector3>("control", 1000);
    motor_sub_ = n_.subscribe("arduino/motors", 1000, &EncoderDebug::motorCallback, this);
  }


  void motorCallback(const gigatron::Motors::ConstPtr& mot_msg) 
  {

    int cmd_left = mot_msg->usec_left - 1500;
    int cmd_right = mot_msg->usec_right - 1500;
    int ticks_left = mot_msg->rpm_left;
    int ticks_right = mot_msg->rpm_right;

    if ((ticks_right <= 0) != (cmd_right <= 0)) {
      ROS_ERROR_STREAM("/arduino/motors: \tleft_ticks\t" << ticks_left << "\tright_ticks\t" << ticks_right << "\tRight encoder direction is wrong!");

      // double motor_revs = ticks_right / 600.0;
      // double wheel_revs = motor_revs * gearRatio;

      // double rpm = wheel_revs * (60.0 * 1000) / S_LOOP_INTERVAL;
      // double vel_right = rpm * RPM_TO_M_S;
    }
    else if ((ticks_left <= 0) != (cmd_left <= 0)) {
      ROS_ERROR_STREAM("/arduino/motors: \tleft_ticks\t" << ticks_left << "\tright_ticks\t" << ticks_right << "\tLeft encoder direction is wrong!");

    }
    else {
      ROS_INFO_STREAM("/arduino/motors: \tleft_ticks\t" << ticks_left << "\tright_ticks\t" << ticks_right);
    }
  
  }

  void publishTickCommands(int left_ticks, int right_ticks) 
  {
    control_.y = left_ticks;
    control_.z = right_ticks;
    ROS_INFO_STREAM("/control: \tlSpC\t" << control_.y << "\trSpC\t" << control_.z);

    control_pub_.publish(control_);

  }

private:
  ros::NodeHandle n_; 
  ros::Publisher control_pub_;
  ros::Subscriber motor_sub_;
  geometry_msgs::Vector3 control_; //$ command message

}; // end of class EncoderDebug

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
  ros::init(argc, argv, "encoder_debug");

  // create a EncoderDebug object to publish and subscribe at same time
  EncoderDebug debugger;

  ros::Rate loop_rate(5);  // run at 1hz

  int i = 0;

  ROS_INFO_STREAM("SPEED!");

  while (ros::ok() && i > -250)
  {
    debugger.publishTickCommands(i, i);

    boost::this_thread::sleep(boost::posix_time::seconds(2));

    ros::spinOnce();
    i = i - 10;
  }
  ROS_INFO_STREAM("Slowing down!");
  while (ros::ok() && i < 250)
  {
    debugger.publishTickCommands(i, i);

    boost::this_thread::sleep(boost::posix_time::seconds(2));

    ros::spinOnce();
    i = i + 10;
  }
  debugger.publishTickCommands(0, 0);

  boost::this_thread::sleep(boost::posix_time::seconds(2));

  ros::spinOnce();
  ROS_INFO_STREAM("DONE!");


  return 0;
}

