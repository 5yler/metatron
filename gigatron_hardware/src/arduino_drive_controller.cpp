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
 * arduino_drive_controller.cpp
 * Gigatron publisher/subscriber for Arduino motor control 
 * 
 * @author  Syler Wagner  <syler@mit.edu>
 *
 * @date    2016-08-10  syler   creation
 * @date    2016-08-17  syler   refactored to take in parameters instead of hardcoding
 *
 * This node listens for gigatron/Drive messages with velocity values and desired steering angle,
 * applies the dynamic model of the car, and publishes ROS msgs to a sketch on the Arduino
 * in order to control the steering and motors.
 **/

#include "ros/ros.h"
#include <boost/thread/thread.hpp>

//$ motor commands
#include <gigatron_hardware/MotorCommand.h>
#include <gigatron/DriveStamped.h>
#include <gigatron/ExtendedState.h>

//$ debugging messages
#include <gigatron_hardware/Radio.h>
#include <gigatron_hardware/Steering.h>
#include <gigatron_hardware/Motors.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>

//$ joint state for wheels
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>


#define PI 3.141592653589793238463

class ArduinoDriveController
{
public:
  ArduinoDriveController()
  {
    //$ get some parameters
    std::string name;
    ros::param::param<std::string>("/car/name", name, "ERROR"); 
    ROS_ERROR("Loading parameters for %s frame...", name.c_str());   

    ros::param::get("/car/max_motor_rpm", _max_motor_rpm);
    ros::param::get("/car/steering_pwm_range", _steering_pwm_range);
    ros::param::get("/car/steering_angle_range", _steering_angle_range);
    _abs_max_steering_angle = 0.5 * _steering_angle_range;

    ros::param::get("/car/gear_ratio", _gear_ratio);
    ros::param::get("/car/wheelbase_width", _wheelbase_width);
    ros::param::get("/car/car_length", _car_length);
    ros::param::get("/car/car_height", _car_height);

    //$ print output
    ROS_WARN("Steering PWM range: %d", _steering_pwm_range);
    ROS_WARN("Steering angle range: %4.2f radians", _steering_angle_range);
    ROS_WARN("Gear ratio: %4.2f", _gear_ratio);

    if (ros::param::has("/car/wheel_radius")) 
    {
      ros::param::get("/car/wheel_radius", _wheel_radius);
      ROS_WARN("Wheel radius: %4.4f", _wheel_radius);

    }
    else if (ros::param::has("/car/wheel_diameter")) 
    {
      double wheel_diameter;
      ros::param::get("/car/wheel_diameter", wheel_diameter);
      ROS_ERROR("Wheel diameter: %4.4f", wheel_diameter);
      _wheel_radius = 0.5 * wheel_diameter;
    }
    else 
    {
      ROS_ERROR("No wheel_radius or wheel_diameter parameter set. Make sure your config files are correct!");
      ROS_WARN("Exiting...");
      ros::shutdown();
    }



    _rpm_to_vel = (2 * PI * _wheel_radius) / 60.0;

    //$ set up ROS subscribers
    drive_sub_ = n_.subscribe("command/drive", 5, &ArduinoDriveController::driveCallback, this);
//    stop_sub_ = n_.subscribe("command/stop", 5, &ArduinoDriveController::stopCallback, this);
    
    motor_sub_ = n_.subscribe("arduino/motors", 5, &ArduinoDriveController::motorCallback, this);
    steer_sub_ = n_.subscribe("arduino/steering", 5, &ArduinoDriveController::steerCallback, this);
    mode_sub_ = n_.subscribe("arduino/mode", 5, &ArduinoDriveController::modeCallback, this);
    estop_sub_ = n_.subscribe("arduino/command/stop", 5, &ArduinoDriveController::estopCallback, this);


    //$ set up ROS publishers
    control_pub_ = n_.advertise<gigatron_hardware::MotorCommand>("arduino/command/motors", 5);
    state_pub_ = n_.advertise<gigatron::ExtendedState>("state", 5);
    joint_pub_ = n_.advertise<sensor_msgs::JointState>("joint_states", 1);
    vis_pub_ = n_.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0);

    mode_ = 0;
    estop_ = false;
    angle_pwm_ = _steering_pwm_range / 2;
    motor_rpm_right_ = motor_rpm_left_ = 0;

    //$ initialize some constant fields for visualization markers
    angle_marker_.header.frame_id = "base_link";
    angle_marker_.id = 0;
    angle_marker_.type = visualization_msgs::Marker::ARROW;
    angle_marker_.action = 0; //$ 0 add/modify an object
    angle_marker_.pose.position.x = _car_length / 2;
    angle_marker_.pose.position.z = _car_height / 2;
    angle_marker_.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    angle_marker_.scale.x = 0.5;
    angle_marker_.scale.y = 0.05;
    angle_marker_.scale.z = 0.05;
    angle_marker_.color.a = 0.5; // Don't forget to set the alpha!
    angle_marker_.color.r = 1.0;
    angle_marker_.color.g = 0.0;
    angle_marker_.color.b = 0.0;

    //$ left wheel marker
    l_wheel_marker_.header.frame_id = "base_link";
    l_wheel_marker_.id = 1;
    l_wheel_marker_.type = visualization_msgs::Marker::ARROW;
    l_wheel_marker_.action = 0; //$ 0 add/modify an object
    l_wheel_marker_.pose.position.x = - 2 * _car_length;
    l_wheel_marker_.pose.position.y = _wheelbase_width;
    l_wheel_marker_.pose.position.z = _car_height / 2;
    l_wheel_marker_.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    l_wheel_marker_.scale.x = 0;
    l_wheel_marker_.scale.y = 0.05;
    l_wheel_marker_.scale.z = 0.05;
    l_wheel_marker_.color.a = 0.3; // Don't forget to set the alpha!
    l_wheel_marker_.color.r = 1.0;
    l_wheel_marker_.color.g = 1.0;
    l_wheel_marker_.color.b = 1.0;

    //$ right wheel marker
    r_wheel_marker_.header.frame_id = "base_link";
    r_wheel_marker_.id = 2;
    r_wheel_marker_.type = visualization_msgs::Marker::ARROW;
    r_wheel_marker_.action = 0; //$ 0 add/modify an object
    r_wheel_marker_.pose.position.x = - 2 * _car_length;
    r_wheel_marker_.pose.position.y = - _wheelbase_width;
    r_wheel_marker_.pose.position.z = _car_height / 2;
    r_wheel_marker_.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    r_wheel_marker_.scale.x = 0;
    r_wheel_marker_.scale.y = 0.05;
    r_wheel_marker_.scale.z = 0.05;
    r_wheel_marker_.color.a = 0.3; // Don't forget to set the alpha!
    r_wheel_marker_.color.r = 1.0;
    r_wheel_marker_.color.g = 1.0;
    r_wheel_marker_.color.b = 1.0;

  }

/*$
  Callback method for Drive messages. The desired steering angle and wheel velocities get translated to servo PWM for steering motor and motor RPM for drive motors. 
 */
  void driveCallback(const gigatron::DriveStamped::ConstPtr& msg) 
  {

    drive_stamp_ = msg->header.stamp;
    double tmp_angle = msg->drive.angle;

    //$ error checking 
    if (tmp_angle > _abs_max_steering_angle) 
    {
      //$ clip commanded angle to upper limit
      tmp_angle  = _abs_max_steering_angle;
    }
    if (tmp_angle < - _abs_max_steering_angle) 
    {
      //$ clip commanded angle to lower limit
      tmp_angle  = - _abs_max_steering_angle;
    }

    int tmp_angle_pwm_cmd = (tmp_angle + _abs_max_steering_angle) * (_steering_pwm_range / _steering_angle_range);

    //$ double error checking for sanity
    if (tmp_angle_pwm_cmd > _steering_pwm_range) 
    {
      //$ clip commanded angle PWM to upper limit
      tmp_angle_pwm_cmd  = _steering_pwm_range;
      ROS_ERROR("tmp_angle_pwm_cmd %d is out of possible PWM range - what are you doing?", tmp_angle_pwm_cmd);
    }
    if (tmp_angle_pwm_cmd < 0) 
    {
      //$ clip commanded angle PWM to lower limit
      tmp_angle_pwm_cmd = 0;
      ROS_ERROR("tmp_angle_pwm_cmd %d is out of possible PWM range - what are you doing?", tmp_angle_pwm_cmd);
    }

    cmd_msg_.angle_command = (uint8_t) tmp_angle_pwm_cmd;

    //$ todo: clip RPM to limits

    if (estop_) {
      cmd_msg_.rpm_left = 0;
      cmd_msg_.rpm_right = 0;
    } else {
      cmd_msg_.rpm_left = msg->drive.vel_left / (_rpm_to_vel * _gear_ratio);
      cmd_msg_.rpm_right = msg->drive.vel_right / (_rpm_to_vel * _gear_ratio);
    }
    control_pub_.publish(cmd_msg_);
    // ROS_INFO("Published PWM %d (%d)", tmp_angle_pwm_cmd, cmd_msg_.angle_command);

    publishDriveVectors(tmp_angle, msg->drive.vel_left, msg->drive.vel_right);

  }

/*$
  Publish the current wheel joint state. 
 */
  void publishDriveVectors(double drive_angle, double v_left, double v_right)
  {
    //E ros::Time now = ros::Time::now();
    angle_marker_.header.stamp = drive_stamp_;// now;
    l_wheel_marker_.header.stamp = drive_stamp_; // now;
    r_wheel_marker_.header.stamp = drive_stamp_; // now;

    angle_marker_.pose.orientation = tf::createQuaternionMsgFromYaw(drive_angle);
    l_wheel_marker_.scale.x = v_left * 0.25;
    r_wheel_marker_.scale.x = v_right * 0.25;

    visualization_msgs::MarkerArray array;

    array.markers.resize(3);
    array.markers[0] = angle_marker_;
    array.markers[1] = l_wheel_marker_;
    array.markers[2] = r_wheel_marker_;

    vis_pub_.publish(array);
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

    if (mode_ == 0) { //$ RC
      //$ make angle marker blue
      angle_marker_.color.r = 0.0;
      angle_marker_.color.b = 1.0;
    } else if (mode_ == 1) { //$ semiautomatic
      //E make angle marker purple
      angle_marker_.color.r = 0.5;
      angle_marker_.color.b = 0.5;
    } else { //$ 2AUTO4U
      // $ make angle marker red
      angle_marker_.color.r = 1.0;
      angle_marker_.color.b = 0.0;
    }
  }

/*$
  Callback method for estop messages from the Arduino. 
 */
  void estopCallback(const std_msgs::Bool::ConstPtr& msg) 
  {
    estop_ = msg->data;
  }

/*$
  Publish the current mode, steering angle and wheel velocities based on information coming in from the Arduino. 
 */
  void publishState()
  {
    state_msg_.header.stamp = drive_stamp_; // ros::Time::now();
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
    state_msg_.drive.angle =  (angle_pwm_ / _steering_pwm_range) * _steering_angle_range - _abs_max_steering_angle;

    //$ convert motor RPM to wheel velocity
    //$ TODO: double check this is correct
    state_msg_.drive.vel_left = motor_rpm_left_ * _rpm_to_vel * _gear_ratio;
    state_msg_.drive.vel_right = motor_rpm_right_ * _rpm_to_vel * _gear_ratio;

    state_msg_.estop = estop_;

    state_pub_.publish(state_msg_);

  }



/*$
  Publish the current wheel joint state. 
 */
  void publishJointState()
  {
    joint_msg_.header.stamp = drive_stamp_; // ros::Time::now();
    //update joint_msg_
    joint_msg_.name.resize(2);
    joint_msg_.position.resize(2);
    joint_msg_.name[0] ="front_left_wheel_joint";
    joint_msg_.position[0] = state_msg_.drive.angle;
    joint_msg_.name[1] ="front_right_wheel_joint";
    joint_msg_.position[1] = state_msg_.drive.angle;

    joint_pub_.publish(joint_msg_);
  }

private:
  ros::NodeHandle n_; 

  ros::Subscriber drive_sub_;
  ros::Subscriber stop_sub_;

  ros::Subscriber motor_sub_;
  ros::Subscriber steer_sub_;
  ros::Subscriber mode_sub_;
  ros::Subscriber estop_sub_;

  ros::Publisher control_pub_;
  ros::Publisher state_pub_;
  ros::Publisher joint_pub_;
  ros::Publisher vis_pub_;
  
  gigatron_hardware::MotorCommand cmd_msg_; //$ command message
  gigatron::ExtendedState state_msg_; //$ state message
  sensor_msgs::JointState joint_msg_;
  ros::Time drive_stamp_;
  visualization_msgs::Marker angle_marker_;
  visualization_msgs::Marker l_wheel_marker_;
  visualization_msgs::Marker r_wheel_marker_;

  /* parameters */
  int _steering_pwm_range;       //$ OK so this isn't actually PWM, but it's the input to the Arduino sketch

  double _steering_angle_range;     //$ [radians]
  double _abs_max_steering_angle;   //$ [radians]
  int _max_motor_rpm;   

  double _gear_ratio;               //$ drive motors / wheels
  double _wheelbase_width;          //$ [m]
  double _car_length;               //$ [m]
  double _car_height;               //$ [m]
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
    controller.publishJointState();
    loop_rate.sleep();
  }

  return 0;
}

