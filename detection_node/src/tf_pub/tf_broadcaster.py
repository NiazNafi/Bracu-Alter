#!/usr/bin/env python

import rospy
import tf
from position_msgs.msg import ObjectPositions

def callback(msg):
    br = tf.TransformBroadcaster()
    for position in msg.object_positions:
        # Create a unique child frame id for each object
        child_frame_id = f"{position.Class}"
        br.sendTransform(
            (position.x/1000, position.z/1000, 0),
            tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time.now(),
            child_frame_id,
            "camera_link"
        )

def listener():
    rospy.init_node('tf_broadcaster', anonymous=True)
    rospy.Subscriber("/objects_position/message", ObjectPositions, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
