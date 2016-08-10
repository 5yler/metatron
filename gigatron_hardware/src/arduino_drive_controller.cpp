 /**
 * arduino_drive_controller.cpp
 * Gigatron publisher/subscriber for Arduino motor control 
 * 
 * @author  Syler Wagner  <syler@mit.edu>
 * @date    2016-08-10    creation
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

//$ motor commands
#include <gigatron_hardware/MotorCommand.h>
#include <gigatron/Drive.h>
#include <gigatron/State.h>

//$ debugging messages
#include <gigatron_hardware/Radio.h>
#include <gigatron_hardware/Steering.h>
#include <gigatron_hardware/Motors.h>

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


class ArduinoDriveController
{
public:
  ArduinoDriveController()
  {
    drive_sub_ = n_.subscribe("command/drive", 1000, &ArduinoDriveController::driveCallback, this);

    motor_sub_ = n_.subscribe("arduino/motors", 1000, &ArduinoDriveController::motorCallback, this);
    steer_sub_ = n_.subscribe("arduino/steering", 1000, &ArduinoDriveController::steerCallback, this);

    control_pub_ = n_.advertise<gigatron_hardware::MotorCommand>("arduino/command/motors", 1000);
    state_pub_ = n_.advertise<gigatron::State>("state", 1000);

    angle_pwm_ = 128;
    motor_rpm_right_ = motor_rpm_left_ = 0;

  }


  void driveCallback(const gigatron::Drive::ConstPtr& msg) 
  {


  }

  void steerCallback(const gigatron_hardware::Steering::ConstPtr& msg) 
  {
    angle_pwm_ = msg->angle;
    //steeringAngle = STEERING_ANGLE_RANGE * (servoPWM / STEERING_PWM_RANGE) - ABS_MAX_STEERING_ANGLE;

  }

  void motorCallback(const gigatron_hardware::Motors::ConstPtr& msg) 
  {
    motor_rpm_left_ = msg->rpm_left;
    motor_rpm_right_ = msg->rpm_right;
    publishState();
  }

  void publishState()
  {
    // state_msg_.mode = ?
    state_msg_.header.stamp = ros::Time::now();

    state_msg_.drive.angle = STEERING_ANGLE_RANGE * (angle_pwm_ / STEERING_PWM_RANGE) - ABS_MAX_STEERING_ANGLE;
    state_msg_.drive.vel_left = motor_rpm_left_ * RPM_TO_M_S * gearRatio;
    state_msg_.drive.vel_right = motor_rpm_right_ * RPM_TO_M_S * gearRatio;
    // state_msg_.header.frame_id = "odom"; ?

    state_pub_.publish(control_msg_);

  }

private:
  ros::NodeHandle n_; 
  ros::Publisher control_pub_;
  ros::Publisher state_pub_;
  ros::Subscriber motor_sub_;
  ros::Subscriber steer_sub_;
  ros::Subscriber drive_sub_;
  gigatron_hardware::MotorCommand control_msg_; //$ command message
  gigatron::State state_msg_; //$ command message
  double angle_pwm_;  //$ current steering angle
  double motor_rpm_right_, motor_rpm_left_;

}; // end of class ArduinoDriveController

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
  ros::init(argc, argv, "arduino_drive_controller");

  // create a ArduinoDriveController object to publish and subscribe at same time
  ArduinoDriveController debugger;

  ros::Rate loop_rate(5);  // run at 1hz

  int i = 0;

  ROS_INFO_STREAM("SPEED!");

  while (ros::ok() && i > -250)
  {
    // debugger.publishTickCommands(i, i);

    boost::this_thread::sleep(boost::posix_time::seconds(2));

    ros::spinOnce();
    i = i - 10;
  }
  ROS_INFO_STREAM("Slowing down!");
  while (ros::ok() && i < 250)
  {
    // debugger.publishTickCommands(i, i);

    boost::this_thread::sleep(boost::posix_time::seconds(2));

    ros::spinOnce();
    i = i + 10;
  }
  // debugger.publishTickCommands(0, 0);

  boost::this_thread::sleep(boost::posix_time::seconds(2));

  ros::spinOnce();
  ROS_INFO_STREAM("DONE!");


  return 0;
}

