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

    if ax_3 > 0.2 and left_b == 0 and right_b == 0:
        msg.data = 'forwards'
    if ax_3 < -0.2 and left_b == 0 and right_b == 0:
        msg.data = 'backwards'
    if ax_2 > 0.2:
        msg.data = 'left'
    if ax_2 < -0.2:
        msg.data = 'right'
    if ax_1 > 0.2 and left_b == 0 and right_b == 0:
        msg.data = 'flipper_forward_up'
    elif ax_1 < -0.5 and left_b == 0 and right_b == 0:
        msg.data = 'flipper_forward_down'
    if ax_4 > 0.2:
        msg.data = 'flipper_backward_up'
    elif ax_4 < -0.5:
        msg.data = 'flipper_backward_down'
    if ax_5 < -0.5:
        msg.data = 'all_flipper_up'
    elif ax_6 < -0.5:
        msg.data = 'all_flipper_down'
    if left_b > 0.5 and ax_3 > 0.2:
        msg.data = 'flipper_left_forward_up'
    elif left_b > 0.5 and ax_3 < -0.2:
        msg.data = 'flipper_left_forward_down'
    if left_b > 0.5 and ax_1 > 0.2:
        msg.data = 'flipper_right_forward_up'
    elif left_b > 0.5 and ax_1 < -0.2:
        msg.data = 'flipper_right_forward_down'
    if right_b > 0.5 and ax_3 > 0.2:
        msg.data = 'flipper_left_backward_up'
    elif right_b > 0.5 and ax_3 < -0.2:
        msg.data = 'flipper_left_backward_down'
    if right_b > 0.5 and ax_1 > 0.2:
        msg.data = 'flipper_right_backward_up'
    elif right_b > 0.5 and ax_1 < -0.2:
        msg.data = 'flipper_right_backward_down'

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

