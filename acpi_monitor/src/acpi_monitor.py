#!/usr/bin/env python
import roslib; roslib.load_manifest('acpi_monitor')
import rospy
import subprocess
import re

from std_msgs.msg import Float32

rospy.init_node('acpi_monitor')
pub = rospy.Publisher('~battery',Float32,latch=True,queue_size=1)
update_period = rospy.get_param("~update_period",5.0)
battery = rospy.get_param("~battery",0)

while not rospy.is_shutdown():
    scan_results = subprocess.check_output(["acpi", "-b"]).split("\n")
    if len(scan_results)>=battery:
        line = [w.strip() for w in scan_results[battery].split(",")] 
        pub.publish(float(line[1].replace("%",".")))
    rospy.sleep(update_period)

