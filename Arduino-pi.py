#!/usr/bin/env python3
import serial
import time
import sys
import tty
import termios
import os

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def clear_screen():
    os.system('clear')

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.reset_input_buffer()
    x, y = 0, 0
    print("Use WASD to move, Q to quit")
    while True:      
        key = get_key()
        if key.lower() == 'q':
            break

        if key.lower() == 'w':
            command = 'w'
        elif key.lower() == 's':
            command = 's'
        elif key.lower() == 'a':
            command = 'a'
        elif key.lower() == 'd':
            command = 'd'
        else:
            continue
        
        ser.write(command.encode('utf-8'))
        line = ser.readline().decode('utf-8').rstrip()
        print(line)
