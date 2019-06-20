#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('floor_nav')
import rospy
from math import *
from task_manager_lib.TaskClient import *
from std_msgs.msg import Bool

rospy.init_node('task_client')
server_node = rospy.get_param("~server","/task_server")
default_period = rospy.get_param("~period",0.05)
tc = TaskClient(server_node,default_period)

end_expl_pub = rospy.Publisher('/exploration_goals/end_expl', Bool)

LowBatt=tc.TaskCheckBatt(foreground=False)
tc.addCondition(ConditionIsCompleted("Low_Battery",tc,LowBatt))

tc.WaitForAuto()
try:
	tc.Constant(duration=3.0,linear=-0.25)
    for angle in [0.0,pi/2,pi,3*pi/2,0.0]:
        tc.Wait(duration=1.0)
        tc.SetHeading(target=angle, max_angular_velocity=1.0)
	tc.TaskExploration()
	tc.PlanTo(goal_x=-0.6,goal_y=0.0,goal_theta=0.0)
	tc.GoToDock()
	
except TaskConditionException, e:
	endExpl = Bool()
	endExpl.data = True
	end_expl_pub.publish(endExpl)
	tc.PlanTo(goal_x=-0.6,goal_y=0.0,goal_theta=0.0)
	tc.GoToDock()
    rospy.logerr("Exception caught: " + str(e))

if not rospy.core.is_shutdown():
    tc.SetManual()

rospy.loginfo("Mission completed")
