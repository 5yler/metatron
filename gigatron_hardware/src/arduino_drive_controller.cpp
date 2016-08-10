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
#include <std_msgs/UInt8.h>


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
    //$ set up ROS subscribers
    drive_sub_ = n_.subscribe("command/drive", 1000, &ArduinoDriveController::driveCallback, this);
    motor_sub_ = n_.subscribe("arduino/motors", 1000, &ArduinoDriveController::motorCallback, this);
    steer_sub_ = n_.subscribe("arduino/steering", 1000, &ArduinoDriveController::steerCallback, this);
    mode_sub_ = n_.subscribe("arduino/mode", 1000, &ArduinoDriveController::modeCallback, this);

    //$ set up ROS publishers
    control_pub_ = n_.advertise<gigatron_hardware::MotorCommand>("arduino/command/motors", 1000);
    state_pub_ = n_.advertise<gigatron::State>("state", 1000);

    mode_ = 0;
    angle_pwm_ = 128;
    motor_rpm_right_ = motor_rpm_left_ = 0;
  }

/*$
  Callback method for Drive messages. The desired steering angle and wheel velocities get translated to servo PWM for steering motor and motor RPM for drive motors. 
 */
  void driveCallback(const gigatron::Drive::ConstPtr& msg) 
  {
    cmd_msg_.angle_command = (msg->angle + ABS_MAX_STEERING_ANGLE) * (STEERING_PWM_RANGE / STEERING_ANGLE_RANGE);

    cmd_msg_.rpm_left = msg->vel_left / (RPM_TO_M_S * gearRatio);
    cmd_msg_.rpm_right = msg->vel_right / (RPM_TO_M_S * gearRatio);
    control_pub_.publish(cmd_msg_);
  }

/*$
  Callback method for Steering messages from the Arduino. 
 */
  void steerCallback(const gigatron_hardware::Steering::ConstPtr& msg) 
  {
    angle_pwm_ = msg->angle;
  }

/*$
  Callback method for Motors messages from the Arduino. 
 */
  void motorCallback(const gigatron_hardware::Motors::ConstPtr& msg) 
  {
    motor_rpm_left_ = msg->rpm_left;
    motor_rpm_right_ = msg->rpm_right;
  }

/*$
  Callback method for mode messages from the Arduino. 
 */
  void modeCallback(const std_msgs::UInt8::ConstPtr& msg) 
  {
    mode_ = msg->data;
  }

/*$
  Publish the current mode, steering angle and wheel velocities based on information coming in from the Arduino. 
 */
  void publishState()
  {
    state_msg_.header.stamp = ros::Time::now();
    // state_msg_.header.frame_id = "odom"; ?

    //$ convert mode int to string
    if (mode_ == 0) {
      state_msg_.mode = "RC";
    } else if (mode_ == 1) {
      state_msg_.mode = "SEMIAUTOMATIC";
    } else if (mode_ == 2) {
      state_msg_.mode = "2AUTO4U";
    } else {
      state_msg_.mode = "WTF?";
    }

    //$ convert PWM to actual steering angle
    state_msg_.drive.angle = STEERING_ANGLE_RANGE * (angle_pwm_ / STEERING_PWM_RANGE) - ABS_MAX_STEERING_ANGLE;

    //$ convert motor RPM to wheel velocity
    //$ TODO: double check this is correct
    state_msg_.drive.vel_left = motor_rpm_left_ * RPM_TO_M_S * gearRatio;
    state_msg_.drive.vel_right = motor_rpm_right_ * RPM_TO_M_S * gearRatio;

    state_pub_.publish(state_msg_);

  }

private:
  ros::NodeHandle n_; 

  ros::Subscriber motor_sub_;
  ros::Subscriber steer_sub_;
  ros::Subscriber drive_sub_;
  ros::Subscriber mode_sub_;

  ros::Publisher control_pub_;
  ros::Publisher state_pub_;
  
  gigatron_hardware::MotorCommand cmd_msg_; //$ command message
  gigatron::State state_msg_; //$ state message
  
  unsigned int mode_;  //$ current mode
  double angle_pwm_;  //$ current steering angle PWM value
  double motor_rpm_right_, motor_rpm_left_; //$ current motor RPM values


}; //$ end of class ArduinoDriveController

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

  ros::Rate loop_rate(10);  // run at 1hz

  int i = 0;

  ROS_INFO_STREAM("SPEED!");

  while (ros::ok())
  {
    ros::spinOnce();
    debugger.publishState();
    loop_rate.sleep();
  }

  return 0;
}

