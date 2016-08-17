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
#include <std_msgs/Bool.h>


#define PI 3.141592653589793238463
#define S_LOOP_INTERVAL 100.0

class ArduinoDriveController
{
public:
  ArduinoDriveController()
  {
    //$ get some parameters
    std::string name;
    ros::param::param<std::string>("~name", name, "ERROR"); 
    ROS_ERROR("Loading parameters for %s frame...", name.c_str());   

    ros::param::get("~steering_pwm_range", _steering_pwm_range);
    ros::param::get("~steering_angle_range", _steering_angle_range);
   _abs_max_steering_angle = 0.5 * _steering_angle_range;

    ros::param::get("~gear_ratio", _gear_ratio);
    ros::param::get("~wheel_radius", _wheel_radius);

    //$ print output
    ROS_WARN("Steering PWM range: %d", _steering_pwm_range);
    ROS_WARN("Steering angle range: %4.2f radians", _steering_angle_range);
    ROS_WARN("Gear ratio: %4.2f", _gear_ratio);
    ROS_WARN("Wheel radius: %4.2f", _wheel_radius);

    _rpm_to_vel = (2 * PI * _wheel_radius) / 60.0;

    //$ set up ROS subscribers
    drive_sub_ = n_.subscribe("command/drive", 5, &ArduinoDriveController::driveCallback, this);
//    stop_sub_ = n_.subscribe("command/stop", 5, &ArduinoDriveController::stopCallback, this);
    
    motor_sub_ = n_.subscribe("arduino/motors", 5, &ArduinoDriveController::motorCallback, this);
    steer_sub_ = n_.subscribe("arduino/steering", 5, &ArduinoDriveController::steerCallback, this);
    mode_sub_ = n_.subscribe("arduino/mode", 5, &ArduinoDriveController::modeCallback, this);

    //$ set up ROS publishers
    control_pub_ = n_.advertise<gigatron_hardware::MotorCommand>("arduino/command/motors", 5);
    state_pub_ = n_.advertise<gigatron::State>("state", 5);

    mode_ = 0;
    estop_ = false;
    angle_pwm_ = _steering_pwm_range / 2;
    motor_rpm_right_ = motor_rpm_left_ = 0;
  }

/*$
  Callback method for Drive messages. The desired steering angle and wheel velocities get translated to servo PWM for steering motor and motor RPM for drive motors. 
 */
  void driveCallback(const gigatron::Drive::ConstPtr& msg) 
  {
    cmd_msg_.angle_command = (msg->angle + _abs_max_steering_angle) * (_steering_pwm_range / _steering_angle_range);

    if (estop_) {
      cmd_msg_.rpm_left = 0;
      cmd_msg_.rpm_right = 0;
    } else {
      cmd_msg_.rpm_left = msg->vel_left / (_rpm_to_vel * _gear_ratio);
      cmd_msg_.rpm_right = msg->vel_right / (_rpm_to_vel * _gear_ratio);
    }
    control_pub_.publish(cmd_msg_);
  }

/*$
  Callback method for estop messages. If estopped, the RPM commands to both drive motors will be zero.
 */
  void stopCallback(const std_msgs::Bool::ConstPtr& msg) 
  {
    // estop_ = msg->data;
    ROS_ERROR("ESTOPPED? %d", estop_);
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
    state_msg_.drive.angle = _steering_angle_range * (angle_pwm_ / _steering_pwm_range) - _abs_max_steering_angle;

    //$ convert motor RPM to wheel velocity
    //$ TODO: double check this is correct
    state_msg_.drive.vel_left = motor_rpm_left_ * _rpm_to_vel * _gear_ratio;
    state_msg_.drive.vel_right = motor_rpm_right_ * _rpm_to_vel * _gear_ratio;

    state_pub_.publish(state_msg_);

  }

private:
  ros::NodeHandle n_; 

  ros::Subscriber drive_sub_;
  ros::Subscriber stop_sub_;

  ros::Subscriber motor_sub_;
  ros::Subscriber steer_sub_;
  ros::Subscriber mode_sub_;

  ros::Publisher control_pub_;
  ros::Publisher state_pub_;
  
  gigatron_hardware::MotorCommand cmd_msg_; //$ command message
  gigatron::State state_msg_; //$ state message

  /* parameters */
  int _steering_pwm_range;       //$ OK so this isn't actually PWM, but it's the input to the Arduino sketch

  double _steering_angle_range;     //$ [radians]
  double _abs_max_steering_angle;   //$ [radians]

  double _gear_ratio;               //$ drive motors / wheels
  double _wheel_radius;             //$ [m]
  double _rpm_to_vel;               //$ conversion factor between wheel rpm to velocity in m/s

  double _state_publish_rate;

  /* current values */
  unsigned int mode_;  //$ current mode
  double angle_pwm_;  //$ current steering angle PWM value
  double motor_rpm_right_, motor_rpm_left_; //$ current motor RPM values
  bool estop_;  //$ estop

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
  ArduinoDriveController controller;

  //$ get state message publishing rate as a parameter
  double state_msg_rate;
  ros::param::param("~state_msg_rate", state_msg_rate, 4.0);
  ros::Rate loop_rate(state_msg_rate);  // run at 1hz

  int i = 0;

  ROS_INFO_STREAM("SPEED!");

  while (ros::ok())
  {
    ros::spinOnce();
    controller.publishState();
    loop_rate.sleep();
  }

  return 0;
}

