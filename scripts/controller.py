#!/usr/bin/env python3
# coding: utf-8
import collections  # <-- Add this line
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from odrive_ros.msg import Status

import odrive
from odrive.enums import *
from odrive.utils import *
import threading

import time
import matplotlib.pyplot as plt
import numpy as np

md = None
odrive_lock = threading.Lock()

# Initialize a deque with a fixed size for the sliding window
window_size = 10  # Adjust this value depending on your requirement
sliding_window = collections.deque(maxlen=window_size)  # <-- Add this line

def OdriveSetup():  
    global md
    with odrive_lock:
        md=odrive.find_any()
        print("Setup Start") 
        time.sleep(1)

        md.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(1)
        md.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(1)
        print("set up is finish")

def LeftCallback(data):
    print("Left CB") 
    with odrive_lock:
        global md
        if md and md.axis1:
            print("Left live") 
            print(data.data)
            md.axis1.controller.input_vel = -data.data

def RightCallback(data):
    print("Right CB") 
    with odrive_lock:
        global md
        if md and md.axis0:
            print("Right live") 
            print(data.data)
            md.axis0.controller.input_vel = data.data

def Controller():
    OdriveSetup()
    # Initialize Node
    rospy.init_node('controller', anonymous=True)
    # Subscriber
    sub1 = rospy.Subscriber('left_rps', Float32, LeftCallback)
    sub2 = rospy.Subscriber('right_rps', Float32, RightCallback)
    # Publisher
    pub = rospy.Publisher('motors_status', Status, queue_size=1)

    rate = rospy.Rate(10)

    time.sleep(2)

    while not rospy.is_shutdown():
        motors_status = Status()
        with odrive_lock:
            if md:
                # Add the current value to the sliding window
                sliding_window.append(md.ibus)  # <-- Add this line
                # Get the peak current as the max of the sliding window
                peak_current = max(sliding_window)  # <-- Add this line
                motors_status.current = np.array(md.ibus, dtype='f8')
                motors_status.s_current = np.array(peak_current, dtype='f8')  # <-- Change this line
                motors_status.voltage = np.array(md.vbus_voltage, dtype='f8')
            if md and md.axis1:
                motors_status.left_pos = np.array(-md.axis1.encoder.pos_estimate % 8192 - 4096, dtype='f8')
                motors_status.left_rps = np.array(-md.axis1.encoder.vel_estimate, dtype='f8')
            if md and md.axis0:
                motors_status.right_pos = np.array(md.axis0.encoder.pos_estimate % 8192 - 4096, dtype='f8')
                motors_status.right_rps = np.array(md.axis0.encoder.vel_estimate, dtype='f8')
        pub.publish(motors_status)
        # errors = dump_errors(md)
        # print("dump_errors:", errors)
        rate.sleep()

if __name__ == '__main__':
    try:
        Controller()
    except rospy.ROSInitException:
        pass