#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('floor_nav')
import rospy
from math import *
from task_manager_lib.TaskClient import *
rom geometry_msgs.msg import PoseStamped

rospy.init_node('task_client')
server_node = rospy.get_param("~server","/task_server")
default_period = rospy.get_param("~period",0.05)
tc = TaskClient(server_node,default_period)

goal_dock_pub = rospy.Publisher('/goal_dock', PoseStamped, queue_size=1)

tc.WaitForAuto()
try:
    tc.Constant(duration=3.0,linear=-0.25)
    #get the pose close to the dock and store it 
	listener = tf.TransformListener()
	((x,y,_),_) = listener.lookupTransform("odom", "/base_link", rospy.Time(0))
	goal_dock = PoseStamped()
	goal_dock.pose.position.x = x
	goal_dock.pose.position.y = y
	goal_dock.pose.position.z = 0
	goal_dock.pose.orientation.x = 0
	goal_dock.pose.orientation.y = 0
	goal_dock.pose.orientation.z = 0
	goal_dock.pose.orientation.w = 1	
	
	goal_dock_pub.publish(goal_dock)
	
    #for angle in [0.0,pi/2,pi,3*pi/2,0.0]:
    #    tc.Wait(duration=1.0)
    #    tc.SetHeading(target=angle,max_angular_velocity=1.5)
		
except TaskException, e:
    rospy.logerr("Exception caught: " + str(e))

if not rospy.core.is_shutdown():
    tc.SetManual()


rospy.loginfo("Mission completed")

