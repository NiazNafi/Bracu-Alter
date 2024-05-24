#!/usr/bin/env python3
import rospy
import tf
import geometry_msgs.msg
import tf2_ros
from tf2_geometry_msgs import PointStamped
from position_msgs.msg import ObjectPositions

def callback(msg, tf_listener):
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transforms = []
    
    try:
        # Get the transform from camera_link to map
        rospy.loginfo("Waiting for transform from 'camera_link' to 'map'...")
        tf_listener.waitForTransform('map', 'camera_link', rospy.Time(0), rospy.Duration(5.0))
        rospy.loginfo("Transform found, looking up transform from 'camera_link' to 'map'...")
        (trans, rot) = tf_listener.lookupTransform('map', 'camera_link', rospy.Time(0))
        rospy.loginfo("Transform successfully looked up.")
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("Could not get transform from 'camera_link' to 'map'")
        return
    
    for i, position in enumerate(msg.object_positions):
        transform_map_to_object = geometry_msgs.msg.TransformStamped()

        transform_map_to_object.header.stamp = rospy.Time.now()
        transform_map_to_object.header.frame_id = "map"
        transform_map_to_object.child_frame_id = f"{position.Class}"

        # Transform the position from camera_link frame to map frame
        point_camera_link = geometry_msgs.msg.PointStamped()
        point_camera_link.header.frame_id = "camera_link"
        point_camera_link.point.x = float(position.x) / 1000
        point_camera_link.point.y = float(position.z) / 1000
        point_camera_link.point.z = 0.0

        # Apply the transformation from camera_link to map
        point_map = tf_listener.transformPoint("map", point_camera_link)

        transform_map_to_object.transform.translation.x = point_map.point.x
        transform_map_to_object.transform.translation.y = point_map.point.y
        transform_map_to_object.transform.translation.z = point_map.point.z
        transform_map_to_object.transform.rotation.x = 0.0
        transform_map_to_object.transform.rotation.y = 0.0
        transform_map_to_object.transform.rotation.z = 0.0
        transform_map_to_object.transform.rotation.w = 1.0
        
        static_transforms.append(transform_map_to_object)

    static_broadcaster.sendTransform(static_transforms)

def listener():
    rospy.init_node('static_tf_publisher', anonymous=True)

    tf_listener = tf.TransformListener()

    rospy.Subscriber("/objects_position/message", ObjectPositions, callback, tf_listener)
    
    rospy.spin()

if __name__ == '__main__':
    listener()
