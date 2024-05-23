#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from position_msgs.msg import ObjectPositions, ObjectPosition

def callback(msg):
    rospy.loginfo("Header: seq = %d, stamp = %d.%d, frame_id = %s", 
                  msg.header.seq, 
                  msg.header.stamp.secs, 
                  msg.header.stamp.nsecs, 
                  msg.header.frame_id)
    
    for position in msg.object_positions:
        rospy.loginfo(f"Object {position.Class}: x = {position.x}, y = {position.y}, z = {position.z}", 
                      )

def listener():
    rospy.init_node('position_listener', anonymous=True)
    rospy.Subscriber("/objects_position/message", ObjectPositions, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

