#! /usr/bin/env python

"""
Individual wheel odometry
dgonz@mit.edu
"""

#Sub to state messages

#publish to a new topic named wheel_distance

import rospy, math
from rospy.rostime import Time
from gigatron.msg import State

distL = 0
distR = 0

def state_callback(msg):
    global distL, distR, lastTime, currentTime
    # find current orientation of robot based on odometry (quaternion coordinates)

    currentTime = rospy.Time.now()
    dL = msg.drive.vel_left
    dR = msg.drive.vel_right

    dT = (currentTime-lastTime).to_sec()
    lastTime = currentTime
    distL += dL*dT
    distR += dR*dT
    rospy.loginfo("distL: %4.1f distR: %4.1f", distL, distR)

if __name__ == '__main__': 
  try:
    rospy.init_node('wheel_distance_pub')
    
    lastTime = rospy.Time.now()
    currentTime = rospy.Time.now()
    state_topic = rospy.get_param('state_topic', '/state') 
    
    rospy.Subscriber(state_topic, State, state_callback, queue_size=1)
    
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
