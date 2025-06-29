#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO

# Set the GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Set variables for the GPIO motor pins
pinMotorAForwards = 10
pinMotorABackwards = 9
pinMotorBForwards = 8
pinMotorBBackwards = 7

# How many times to turn the pin on and off each second
Frequency = 20
# How long the pin stays on each cycle, as a percent (here, it's 30%)
DutyCycle = 30
# Setting the duty cycle to 0 means the motors will not turn
Stop = 0

# Set the GPIO Pin mode to be Output
GPIO.setup(pinMotorAForwards, GPIO.OUT)
GPIO.setup(pinMotorABackwards, GPIO.OUT)
GPIO.setup(pinMotorBForwards, GPIO.OUT)
GPIO.setup(pinMotorBBackwards, GPIO.OUT)

# Set the GPIO to software PWM at 'Frequency' Hertz
pwmMotorAForwards = GPIO.PWM(pinMotorAForwards, Frequency)
pwmMotorABackwards = GPIO.PWM(pinMotorABackwards, Frequency)
pwmMotorBForwards = GPIO.PWM(pinMotorBForwards, Frequency)
pwmMotorBBackwards = GPIO.PWM(pinMotorBBackwards, Frequency)

# Start the software PWM with a duty cycle of 0 (i.e. not moving)
pwmMotorAForwards.start(Stop)
pwmMotorABackwards.start(Stop)
pwmMotorBForwards.start(Stop)
pwmMotorBBackwards.start(Stop)

# Turn all motors off
def StopMotors():
   pwmMotorAForwards.ChangeDutyCycle(Stop)
   pwmMotorABackwards.ChangeDutyCycle(Stop)
   pwmMotorBForwards.ChangeDutyCycle(Stop)
   pwmMotorBBackwards.ChangeDutyCycle(Stop)

# Turn both motors forwards
def Forwards():
   pwmMotorAForwards.ChangeDutyCycle(DutyCycle)
   pwmMotorABackwards.ChangeDutyCycle(Stop)
   pwmMotorBForwards.ChangeDutyCycle(DutyCycle)
   pwmMotorBBackwards.ChangeDutyCycle(Stop)

# Turn both motors backwards
def Backwards():
   pwmMotorAForwards.ChangeDutyCycle(Stop)
   pwmMotorABackwards.ChangeDutyCycle(DutyCycle)
   pwmMotorBForwards.ChangeDutyCycle(Stop)
   pwmMotorBBackwards.ChangeDutyCycle(DutyCycle)

# Turn left
def Left():
   pwmMotorAForwards.ChangeDutyCycle(Stop)
   pwmMotorABackwards.ChangeDutyCycle(DutyCycle)
   pwmMotorBForwards.ChangeDutyCycle(DutyCycle)
   pwmMotorBBackwards.ChangeDutyCycle(Stop)

# Turn Right
def Right():
   pwmMotorAForwards.ChangeDutyCycle(DutyCycle)
   pwmMotorABackwards.ChangeDutyCycle(Stop)
   pwmMotorBForwards.ChangeDutyCycle(Stop)
   pwmMotorBBackwards.ChangeDutyCycle(DutyCycle)

# Message handler
def CommandCallback(commandMessage):
   command = commandMessage.data
   if command == 'forwards':
       print('Moving forwards')
       Forwards()
   elif command == 'backwards':
       print('Moving backwards')
       Backwards()
   elif command == 'left':
       print('Turning left')
       Left()
   elif command == 'right':
       print('Turning right')
       Right()
   elif command == 'stop':
       print('Stopping')
       StopMotors()
   else:
       print('Unknown command, stopping instead')
       StopMotors()

rospy.init_node('driver')

rospy.Subscriber('/command', String, CommandCallback)

rospy.spin()
print('Shutting down: stopping motors')
StopMotors()
GPIO.cleanup()
