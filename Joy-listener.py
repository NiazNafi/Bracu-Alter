#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy

def joy_callback(data):
    # Extract the values of the axes
    axis_1 = data.axes[0]
    axis_2 = data.axes[1]
    axis_3 = data.axes[2]
    axis_4 = data.axes[3]

    # Extract the values of the buttons
    button_1 = data.buttons[0]
    button_2 = data.buttons[1]
    button_3 = data.buttons[2]
    button_4 = data.buttons[3]
    button_5 = data.buttons[4]

    # Print the values (or handle them as needed)
    rospy.loginfo(f"Axis 1: {axis_1}, Axis 2: {axis_2}, Axis 3: {axis_3}, Axis 4: {axis_4}")
    rospy.loginfo(f"Button 1: {button_1}, Button 2: {button_2}, Button 3: {button_3}, Button 4: {button_4}, Button 5: {button_5}")

def joy_listener():
    rospy.init_node('joy_listener', anonymous=True)
    rospy.Subscriber("/joy", Joy, joy_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        joy_listener()
    except rospy.ROSInterruptException:
        pass
