#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# Author: Simon Le Goff

# This ROS Node converts Joystick inputs from the joy node
# into commands for the robot in vrep

# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Twist commands
# axis 1 aka left stick vertical controls linear speed
# axis 0 aka left stick horizonal controls angular speed
def callback(data):
    twist = Twist()
    twist.linear.x = 0.3*data.axes[1]
    twist.angular.z = 0.3*data.axes[0]
    pub.publish(twist)

# Intializes everything
def start():
    rospy.init_node('Joy2Robot')
    # publishing to "/vrep/twistCommad" to control the robot in vrep
    global pub
    
    pub = rospy.Publisher('/vrep/twistCommand', Twist,queue_size=100)
    #pub = rospy.Publisher('/teleop/cmd_vel', Twist)
    
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("/joy", Joy, callback)
    # starts the node
    
    rospy.spin()

if __name__ == '__main__':
    start()
