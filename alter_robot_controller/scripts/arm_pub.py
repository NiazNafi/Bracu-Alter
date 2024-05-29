#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String

def joy_callback(data):
    # Extract the values of the axes
    ax_1 = data.axes[3]
    ax_2 = data.axes[2]
    ax_3 = data.axes[1]
    ax_4 = data.axes[0]
    ax_5 = data.axes[4]
    ax_6 = data.axes[5]

    # Extract the values of the buttons
    left_b = data.buttons[6]
    right_b = data.buttons[7]

    msg = String()

    if ax_3 > 0.2:
        msg.data = 'j1u'
    if ax_3 < -0.2:
        msg.data = 'j1d'
    if ax_2 > 0.2:
        msg.data = 'j3u'
    if ax_2 < -0.2:
        msg.data = 'j3d'
    if ax_1 > 0.2:
        msg.data = 'j2u'
    elif ax_1 < -0.5:
        msg.data = 'j2d'
    if ax_4 > 0.2:
        msg.data = 'br'
    elif ax_4 < -0.5:
        msg.data = 'bl'
    if ax_5 < -0.5:
        msg.data = 'clwu'
    elif ax_6 < -0.5:
        msg.data = 'clwd'
    if left_b > 0.5 :
        msg.data = 'j4u'
    elif left_b > 0.5:
        msg.data = 'j4d'

    # Publish the message
    pub.publish(msg)

def joy_listener():
    global pub
    rospy.init_node('joy_listener', anonymous=True)
    pub = rospy.Publisher('/armcmd', String, queue_size=10)
    rospy.Subscriber("/joy", Joy, joy_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        joy_listener()
    except rospy.ROSInterruptException:
        pass

