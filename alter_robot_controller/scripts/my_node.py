#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
nodename="Ar_sub"
topicname="r_a"
def callbackFunction(message):
	rospy.loginfo(f"from Arduino{message.data}")
rospy.init_node("Ar_sub",anonymous=True)
rospy.Subscriber(topicname,String,callbackFunction)
rospy.spin()