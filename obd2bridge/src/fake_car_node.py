#!/usr/bin/python3
#
# obdii_logger.py
#
# This python3 program sends out OBDII request then logs the reply to the sd card.
# For use with PiCAN boards on the Raspberry Pi
# http://skpang.co.uk/catalog/pican2-canbus-board-for-raspberry-pi-2-p-1475.html
#
# Make sure Python-CAN is installed first http://skpang.co.uk/blog/archives/1220
#
#  24-08-16 SK Pang
#


# The MIT License (MIT)

# Copyright (c) 2016 Sukkin Pang

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


# import RPi.GPIO as GPIO
import can
import time
import os
import queue
import rospy
import sys
from threading import Thread

# led = 22
# GPIO.setmode(GPIO.BCM)
# GPIO.setwarnings(False)
# GPIO.setup(led,GPIO.OUT)
# GPIO.output(led,True)

# For a list of PIDs visit https://en.wikipedia.org/wiki/OBD-II_PIDs
ENGINE_COOLANT_TEMP = 0x05
ENGINE_RPM = 0x0C
VEHICLE_SPEED = 0x0D
MAF_SENSOR = 0x10
O2_VOLTAGE = 0x14
THROTTLE = 0x11

PID_REQUEST = 0x7DF
PID_REPLY = 0x7E8

outfile = open('log.txt', 'w')

can_interface = 'vcan0'
bus = ''
print('\n\rCAN Rx test')
print('Bring up CAN0....')
q = queue.Queue()
# Bring up can0 interface at 500kbps
# os.system("sudo /sbin/ip link set vcan0 up type can bitrate 500000")
time.sleep(0.1)
print('Ready')


def can_rx_task():  # Receive thread
    while True:
        message = bus.recv()
        if message.arbitration_id == PID_REQUEST:
            q.put(message)			# Put message into queue


def obd():
    print("started obd2 fake car")

    rx = Thread(target=can_rx_task)
    rx.start()
    temperature = 0
    rpm = 0
    speed = 0
    throttle = 0
    c = ''
    count = 0
    try:
        while True:
            for i in range(4):
                while (q.empty() == True):  # Wait until there is a message
                    pass
                message = q.get()

                if message.arbitration_id == PID_REQUEST and message.data[2] == ENGINE_COOLANT_TEMP:
                    msg = can.Message(arbitration_id=PID_REPLY, data=[
                                      0x02, 0x01, ENGINE_COOLANT_TEMP, 0x20, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)
                    bus.send(msg)
                    time.sleep(0.05)

                if message.arbitration_id == PID_REQUEST and message.data[2] == ENGINE_RPM:
                    msg = can.Message(arbitration_id=PID_REPLY, data=[
                                      0x02, 0x01, ENGINE_RPM, 0x20, 0x10, 0x00, 0x00, 0x00], is_extended_id=False)
                    bus.send(msg)
                    time.sleep(0.05)

                if message.arbitration_id == PID_REQUEST and message.data[1] == 0x22:
                    msg = can.Message(arbitration_id=PID_REPLY, data=[
                                      0x05, 0x62, message.data[3], message.data[4], 0xff, 0x00, 0x00, 0x00], is_extended_id=False)
                    bus.send(msg)
                    time.sleep(0.05)

                if message.arbitration_id == PID_REQUEST and message.data[2] == VEHICLE_SPEED:
                    msg = can.Message(arbitration_id=PID_REPLY, data=[
                                      0x02, 0x01, VEHICLE_SPEED, 0x43, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)
                    bus.send(msg)
                    time.sleep(0.05)

                if message.arbitration_id == PID_REQUEST and message.data[2] == THROTTLE:
                    msg = can.Message(arbitration_id=PID_REPLY, data=[
                                      0x02, 0x01, THROTTLE, 0x40, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)
                    bus.send(msg)
                    time.sleep(0.05)

            count += 1

    except KeyboardInterrupt:
        outfile.close()		# Close logger file
        print('\n\rKeyboard interrtupt')

# Main function, takes 1 argument: name of interface i.e. "vcan0" or "can0"(only in loopback mode)
if __name__ == '__main__':
    try:
        print(sys.argv)
        if (len(sys.argv) >= 2):
            can_interface = sys.argv[1]
        try:
            if (can_interface == 'vcan0'):
                bus = can.interface.Bus(
                    channel=can_interface, interface='socketcan')
            elif (can_interface == 'can0'):
                can.interface.Bus(channel='can0', bustype='socketcan_native')
            else:
                exit()
        except OSError:
            print('Cannot find PiCAN board.')
            exit()
        obd()
    except rospy.ROSInterruptException:
        pass
