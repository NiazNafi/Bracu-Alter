#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt16, String  # Added String import

class JoystickToPWM:
    def __init__(self):
        rospy.init_node('joystick_pwm_controller')
        
        # PWM publisher for rosserial
        self.pwm_pub = rospy.Publisher('/pwm_commands', UInt16, queue_size=10)
        
        # String publisher for debugging
        self.debug_pub = rospy.Publisher('/command', String, queue_size=10)
        
        # Joystick mapping configuration
        self.config = {
            'axes': {
                'right_forward': 4,  # ax_5
                'left_forward': 5    # ax_6
            },
            'buttons': {
                'right_backward': 6,
                'left_backward': 7
            },
            'threshold': 0.2,
            'pwm_range': (0, 255)  # Min, Max PWM values
        }
        
        rospy.loginfo("Joystick PWM Controller initialized")

    def map_value(self, value, in_min, in_max, out_min, out_max):
        """Map joystick value to PWM range"""
        # Clamp the input value first
        value = max(min(value, in_max), in_min)
        return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

    def joy_callback(self, data):
        """Process joystick data and generate PWM commands"""
        # Get current states
        ax_5 = data.axes[self.config['axes']['right_forward']]
        ax_6 = data.axes[self.config['axes']['left_forward']]
        left_b = data.buttons[self.config['buttons']['left_backward']]
        right_b = data.buttons[self.config['buttons']['right_backward']]
        
        pwm_msg = UInt16()
        debug_msg = String()
        pwm_value = 0
        
        # Right motor control
        if ax_5 < -self.config['threshold']:
            # Map from [-1.0 to -threshold] to [127 to 255]
            pwm_value = self.map_value(
                abs(ax_5), 
                self.config['threshold'], 
                1.0, 
                127, 
                self.config['pwm_range'][1]
            )
            debug_msg.data = f'right_forward:{pwm_value}'
            pwm_msg.data = pwm_value
        elif right_b > 0.5:
            # Full reverse
            debug_msg.data = 'right_backward:255'
            pwm_msg.data = 255  # Full reverse PWM
            
        # Left motor control
        elif ax_6 < -self.config['threshold']:
            pwm_value = self.map_value(
                abs(ax_6),
                self.config['threshold'],
                1.0,
                127,
                self.config['pwm_range'][1]
            )
            debug_msg.data = f'left_forward:{pwm_value}'
            pwm_msg.data = pwm_value
        elif left_b > 0.5:
            debug_msg.data = 'left_backward:255'
            pwm_msg.data = 255  # Full reverse PWM
            
        else:
            # No movement - neutral position
            debug_msg.data = 'neutral:0'
            pwm_msg.data = 0
            
        # Publish messages
        self.pwm_pub.publish(pwm_msg)
        self.debug_pub.publish(debug_msg)

def main():
    controller = JoystickToPWM()
    rospy.Subscriber("/joy", Joy, controller.joy_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Controller node terminated")