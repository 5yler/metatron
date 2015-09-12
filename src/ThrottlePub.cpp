/**
 * ThrottlePub.cpp
 * Gigatron publisher for servo control using rosserial and Arduino to emulate throttle
 * 
 * @author  Syler Wagner  <syler@mit.edu>
 * @date    2015-07-04    creation
 *
 * This node publishes ROS msgs subscribed to by the listener sketch on the Arduino
 * in order to control the servo.
 * 
 * Usage Instructions:
 * 1. Startup your roscore and the rosserial python node in their own terminal windows.
 *  roscore
 *  rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0
 * 2. Start ThrottlePub in a separate window to control the servo.
 *  rosrun gigatron ThrottlePub
 **/

#include "ros/ros.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"


/**
 * This node demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
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
  ros::init(argc, argv, "ThrottlePub");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher control_pub = n.advertise<std_msgs::Int16MultiArray>("control", 1000);

  /**
   * A ros::Rate object allows you to specify a frequency that you would 
   * like to loop at. It will keep track of how long it has been since 
   * the last call to Rate::sleep(), and sleep for the correct amount of time.
   */
  ros::Rate loop_rate(10);  // run at 10hz

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;


  const unsigned int data_sz = 3;
  /**
   * This is a message object. You stuff it with data, and then publish it.
   */
  std_msgs::Int16MultiArray m;

  m.layout.dim.push_back(std_msgs::MultiArrayDimension());
  m.layout.dim[0].size = data_sz;
  m.layout.dim[0].stride = 1;
  m.layout.dim[0].label = "controls";

  // only needed if you don't want to use push_back
  m.data.resize(data_sz);

  m.data[0] = 0;
  m.data[1] = 0;
  m.data[2] = 0;

  while (ros::ok() && count < 100)
  {

    std_msgs::Int16MultiArray msg = m;
    msg.data[1] = count;

    // ROS_INFO_STREAM is a replacement for cout
    ROS_INFO_STREAM("Left motor PWM command: " << msg.data[1]);
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    control_pub.publish(msg);

    /** 
     * Calling ros::spinOnce() here is not necessary for this simple program, 
     * because we are not receiving any callbacks. However, if you were to 
     * add a subscription into this application, and did not have ros::spinOnce() 
     * here, your callbacks would never get called. So, add it for good measure.
     */
    ros::spinOnce();

    /**
     * Now we use the ros::Rate object to sleep for the time remaining to let us 
     * hit our 10hz publish rate.
     */
    loop_rate.sleep();
    ++count;
  }

  ROS_INFO_STREAM("Slowing down!");
  while (ros::ok() && count > 0)   // count down
  {

    std_msgs::Int16MultiArray msg = m;
    msg.data[1] = count;

    ROS_INFO_STREAM("Left motor PWM command: " << msg.data[1]);
    control_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
    --count;
  }
  ROS_INFO_STREAM("SPEED!");
  while (ros::ok() && count < 100)   // count up again, but faster
  {
    std_msgs::Int16MultiArray msg = m;
    msg.data[2] = count;

    ROS_INFO_STREAM("Right motor PWM command: " << msg.data[2]);
    control_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
    count = count + 2;
  }
  ROS_INFO_STREAM("Slowing down again, but FASTER!");
  while (ros::ok() && count >= 0)   // count down again, but faster
  {
    std_msgs::Int16MultiArray msg = m;
    m.sgdata[2] = count;

    ROS_INFO_STREAM("Right motor PWM command: " << msg.data[2]);
    control_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
    count = count - 2;
  }

  ROS_INFO_STREAM("DONE!");

  return 0;
}
