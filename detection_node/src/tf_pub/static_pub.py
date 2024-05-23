#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
from position_msgs.msg import ObjectPositions

def callback(msg):
    br = tf2_ros.StaticTransformBroadcaster()
    static_transforms = []
    
    for position in msg.object_positions:
        static_transform = geometry_msgs.msg.TransformStamped()
        
        static_transform.header.stamp = rospy.Time.now()
        static_transform.header.frame_id = "camera_link"
        static_transform.child_frame_id = f"{position.Class}"
        
        static_transform.transform.translation.x = float(position.x)/1000
        static_transform.transform.translation.y = float(position.z)/1000
        static_transform.transform.translation.z = 0.0
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0
        
        static_transforms.append(static_transform)
    
    br.sendTransform(static_transforms)

def listener():
    rospy.init_node('static_tf_broadcaster', anonymous=True)
    rospy.Subscriber("/objects_position/message", ObjectPositions, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
