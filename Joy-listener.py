#!/usr/bin/env python3 

import rospy
from sensor_msgs.msg import Joy

def joy_callback(data):
    # Extract the values of the axes
    ax_1 = data.axes[3]
    ax_2 = data.axes[2]
    ax_3 = data.axes[1]
    ax_4 = data.axes[0]

    # Extract the values of the buttons
    button_1 = data.buttons[0]
    button_2 = data.buttons[1]
    button_3 = data.buttons[2]
    button_4 = data.buttons[3]
    button_5 = data.buttons[4]
    if ax_3>0.2:
        rospy.loginfo('Forward')
    if ax_3<-0.2:
        rospy.loginfo('Backward')
    if ax_2>0.2:
        rospy.loginfo("left")
    if ax_2<-0.2:
        rospy.loginfo("right")
    if ax_1>0.2:
        rospy.loginfo("Flipper Forward Up")
    if ax_2>-0.5:
        rospy.loginfo("Flipper Forward Down")
    if ax_4>0.2:
        rospy.loginfo("Flipper BackWard Up")
    if ax_4>-0.5:
        rospy.loginfo("Flipper BackWard Down")
    



    # Print the values (or handle them as needed)
    #rospy.loginfo(f"Axis 1: {ax_1}, Axis 2: {ax_2}, Axis 3: {ax_3}, Axis 4: {ax_4}")
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
