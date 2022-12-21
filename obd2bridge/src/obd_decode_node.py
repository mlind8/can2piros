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
# Changes by:
# Mattias Lind 2022
# Zachary McCormick 2022

import can
import time
import os
import queue
import rospy
import sys
from obd2msg.msg import Obd2msg
import threading

# For a list of PIDs visit https://en.wikipedia.org/wiki/OBD-II_PIDs
ENGINE_RPM = 0x0C
VEHICLE_SPEED = 0x0D
MAF_SENSOR = 0x10
O2_VOLTAGE = 0x14
THROTTLE = 0x11

PRIUS_TEST1 = 0x10
PRIUS_TEST2 = 0x04
PID_REQUEST = 0x7DF
PID_REPLY = 0x7E8

TRANSMIT_RATE = 10  # Hz

logfile_name = "log.txt"

can_interface = 'vcan0'

q = queue.Queue()


# Bring up can0 interface at 500kbps saved because I sometimes forget the command
# os.system("sudo /sbin/ip link set vcan0 up type can bitrate 500000")
# time.sleep(0.1)
# print('Ready')
bus = ''
replay_file = ''


# A thread that can be stopped an can receive any function as a parameter to run until the stop flag is set
class StoppableThread(threading.Thread):
    """Thread class with a stop() method. The thread itself has to check
    regularly for the stopped() condition."""

    def __init__(self, func, *args, **kwargs):
        super(StoppableThread, self).__init__()
        self._stop_event = threading.Event()
        # pass the _stop_event to the function func
        self.run = lambda: func(self._stop_event, *args, **kwargs)

    def stop(self):
        self._stop_event.set()


def can_rx_task(event):  # Receive thread
    while not event.is_set():
        message = bus.recv(timeout=0.2)
        if message and message.arbitration_id == PID_REPLY:
            q.put(message)			# Put message into queue


def can_tx_task(event, tx_rate):  # Transmit thread
    msg_engine_rpm = can.Message(arbitration_id=PID_REQUEST, data=[
                                    0x02, 0x01, ENGINE_RPM, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)
    msg_vehicle_speed = can.Message(arbitration_id=PID_REQUEST, data=[
                                    0x02, 0x01, VEHICLE_SPEED, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)
    msg_throttle = can.Message(arbitration_id=PID_REQUEST, data=[
                                    0x02, 0x01, THROTTLE, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)
    while not event.is_set():
        try:
            # Sent a Engine RPM request
            bus.send(msg_engine_rpm)

            # Sent a Vehicle speed  request
            bus.send(msg_vehicle_speed)

            # Sent a Throttle position request
            bus.send(msg_throttle)
        except:
            # Bus is not connected
            pass
        time.sleep(1.0/tx_rate)


def replay():
    print("replaying")
    try:
        with open(replay_file, "r") as rfile:
            oldt = 0
            pub = rospy.Publisher("obd2msg", Obd2msg, queue_size=10)
            rospy.init_node("obd2")
            rate = rospy.Rate(10)
            loopvar = True

            while loopvar:
                line = rfile.readline()
                if line:
                    line = line.replace('\n', '')
                    parts = line.split(',')
                    ntime = float(parts[0])
                    if oldt == 0:
                        time.sleep(0)
                    else:
                        time.sleep(ntime - oldt)
                    oldt = ntime

                    pubmsg = Obd2msg(parts[1], parts[2], parts[3])
                    print(pubmsg)
                    pub.publish(pubmsg)
                    # print("{}\nnewline".format(line));
                else:
                    loopvar = False
    except Exception as e:
        # Catch keyboard interrupt
        rfile.close()		# Close logger file
        print('\n\rKeyboard interrtupt')
        print(e)


def obd():
    global rx_tsk_run
    global tx_tsk_run
    pub = rospy.Publisher("obd2msg", Obd2msg, queue_size=10)
    rospy.init_node("obd2")
    rate = rospy.Rate(TRANSMIT_RATE)
    print("started obd2 logger")
    useful_pid = [0] * (0xff*0xff)
    rx = StoppableThread(can_rx_task)
    rx.start()
    tx = StoppableThread(can_tx_task, TRANSMIT_RATE)
    tx.start()
    temperature = 0
    rpm = 0
    speed = 0

    throttle = 0
    count = 0
    with open(logfile_name, "a") as logfile:
        while not rospy.is_shutdown():
            try:
                if not q.empty():  # Wait until there is a message
                    message = q.get()

                    if message.arbitration_id == PID_REPLY and message.data[2] == ENGINE_RPM:
                        # Convert data to RPM
                        rpm = round(((message.data[3]*256) + message.data[4])/4)

                    if message.arbitration_id == PID_REPLY and message.data[2] == VEHICLE_SPEED:
                        speed = message.data[3]										# Convert data to km
                    if message.arbitration_id == PID_REPLY and message.data[2] == THROTTLE:
                        throttle = round(
                            (message.data[3]*100)/255)					# Conver data to %

                    pubmsg = Obd2msg(rpm, speed, throttle)
                    # Timestamp the pubmsg
                    pubmsg.header.stamp = rospy.Time.now()
                    logmsg = "{},{},{},{}\n".format(
                        time.time(), rpm, speed, throttle)
                    print(pubmsg)
                    logfile.write(logmsg)
                    #            print(c,file = outfile) # Save data to file
                    count += 1
                    pub.publish(pubmsg)
                    print(pubmsg)
                rate.sleep()
            except:
                print("Keyboard interrupt")
                break
    print("Stopping threads")
    tx.stop()
    rx.stop()
    tx.join()
    rx.join()


if __name__ == '__main__':
    try:
        if (len(sys.argv) >= 3 and sys.argv[1] == "replay"):  # replay mode
            replay_file = sys.argv[2]
          #  timen = time.time();
           # time.sleep(3);
            # timed = time.time() - timen ;
            # print("{},{}".format(timen,timed));
            replay()
        elif len(sys.argv) >= 2:  # collection mode
            can_interface = sys.argv[1]
            try:
                bus = can.interface.Bus(
                    channel=can_interface, interface='socketcan')
            except OSError:
                print('Cannot find PiCAN board.')
            if len(sys.argv) >= 3:
                logfile_name = sys.argv[2]
            obd()
            print("exited obd")
        else:  # print usage
            print("Faulty usage\nArguments:\n1:\'replay\' + replay_file\n2:can_link (i.e. \'vcan0', \'can0 etc.) optional: log_file")
    except rospy.ROSInterruptException:
        pass
