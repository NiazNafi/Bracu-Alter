#!/usr/bin/env python3 

import rospy
from sensor_msgs.msg import Joy

def joy_callback(data):
    # Extract the values of the axes
    ax_1 = data.axes[3]
    ax_2 = data.axes[2]
    ax_3 = data.axes[1]
    ax_4 = data.axes[0]
    ax_5=data.axes[4]
    ax_6=data.axes[5]

    # Extract the values of the buttons
    left_b = data.buttons[6]
    right_b = data.buttons[7]

    if ax_3>0.2 and left_b==0 and right_b==0:
        rospy.loginfo('Forward')
    if ax_3<-0.2 and left_b==0 and right_b==0:
        rospy.loginfo('Backward')
    if ax_2>0.2:
        rospy.loginfo("left")
    if ax_2<-0.2:
        rospy.loginfo("right")
    if ax_1>0.2 and left_b==0 and right_b==0:
        rospy.loginfo("Flipper Forward Up")
    elif ax_1<-0.5 and left_b==0 and right_b==0:
        rospy.loginfo("Flipper Forward Down")
    if ax_4>0.2:
        rospy.loginfo("Flipper BackWard Up")
    elif ax_4<-0.5:
        rospy.loginfo("Flipper BackWard Down")
    if ax_5<-0.5:
        rospy.loginfo("All Flipper Up")
    elif ax_6<-0.5:
        rospy.loginfo("All Flipper Down")
    if left_b>0.5 and ax_3>0.2:
        rospy.loginfo("flipper left For Up")
    elif left_b>0.5 and ax_3<-0.2:
        rospy.loginfo("flipper left For Down")
    if left_b>0.5 and ax_1>0.2:
        rospy.loginfo("flipper Right For UP")
    elif left_b>0.5 and ax_1<-0.2:
        rospy.loginfo("flipper Right For Down")
    if right_b>0.5 and ax_3>0.2:
        rospy.loginfo("flipper left Back Up")
    elif right_b>0.5 and ax_3<-0.2:
        rospy.loginfo("flipper left Back Down")
    if right_b>0.5 and ax_1>0.2:
        rospy.loginfo("flipper Right Back UP")
    elif right_b>0.5 and ax_1<-0.2:
        rospy.loginfo("flipper Right Back Down")

def joy_listener():
    rospy.init_node('joy_listener', anonymous=True)
    rospy.Subscriber("/joy", Joy, joy_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        joy_listener()
    except rospy.ROSInterruptException:
        pass
