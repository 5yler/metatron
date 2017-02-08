#! /usr/bin/env python

'''
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
  
'''

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
