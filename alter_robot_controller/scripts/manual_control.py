#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String

def joy_callback(data):
    # Extract the values of the axes
    ax_5 = data.axes[4]
    ax_6 = data.axes[5]

    # Extract the values of the buttons
    left_b = data.buttons[6]
    right_b = data.buttons[7]

    msg = String()


    if ax_5 < -0.5:
        msg.data = 'right_forward'
    elif left_b > 0.5:
        msg.data = 'left_backward'
    if ax_6 < -0.5:
        msg.data = 'left_forward'
    elif right_b > 0.5:
        msg.data = 'right_backward'

    # Publish the message
    pub.publish(msg)

def joy_listener():
    global pub
    rospy.init_node('joy_listener', anonymous=True)
    pub = rospy.Publisher('/command', String, queue_size=10)
    rospy.Subscriber("/joy", Joy, joy_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        joy_listener()
    except rospy.ROSInterruptException:
        pass
