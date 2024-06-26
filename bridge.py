#!/usr/bin/env python3

# This program acts as bridge between control_motor (in raspberry) and
# plot (in PC). It has two threads, that communicate between them with
# a FIFO. This two threads have normal priority, they are not real-time
# threads.

# thread3 (UNIX socket client): receives real time data from motor control process (in C, server, thread1)
# and stores it in a FIFO queue that is afterwards emptied by thread4_zmq_publisher.

# thread4 (ZMQ publisher). This thread gets data from the FIFO queue and
# sends it to host computer using ZMQ publisher-subscriber paradigm.

from ctypes import *
import time
import socket
import struct
import os
import queue
import zmq

# Declare queue (FIFO) for communication between thread3_client and thread4_zmq_publisher
fifo = queue.Queue(1000)

# Definition of constants related to threading
PTHREAD_STACK_MIN = 131072

SCHED_FIFO = 1

TIMER_ABSTIME = 1
CLOCK_REALTIME = 0
CLOCK_MONOTONIC_RAW	= 4

NSEC_PER_SEC = 1000000000

interval = 500000000

# The following code allows using some threading related C types in python

lc = CDLL('libc.so.6')

class pthread_attr_t(Union):
    _fields_ = [('__size', c_char*64),
                ('__aling', c_int)]

class sched_param(Structure):    
    _fields_ = [('sched_priority', c_int)]

class timeval(Structure):
    _fields_ = [('t_sec', c_long),
                ('t_nsec', c_long)]

# Create global variables needed to issue threads
attr = pthread_attr_t()
param = sched_param()
thread3 = c_void_p()
thread4 = c_void_p()
t_read = timeval()

# The struct timespec consists of nanoseconds and seconds. If the nanoseconds are getting
# bigger than 1000000000 (= 1 second) the variable containing seconds has to be
# incremented and the nanoseconds decremented by 1000000000.
def tsnorm(ts):
    while(ts.t_nsec >= NSEC_PER_SEC):
        ts.t_nsec = ts.t_nsec-NSEC_PER_SEC
        ts.t_sec = ts.t_sec+1
    return ts

# Function associated to thread3 (UNIX socket client). This thread receives real time data from motor
# control process (in C, server, thread1_control) and stores it in a FIFO queue that is afterwards
# emptied by thread4_zmq_publisher.
def thread3_client(data):
    # Set the path for the Unix socket
    socket_path = '/tmp/control_socket'
    
    while True:
        # Create the Unix socket client
        client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        # Connect to the server
        client.connect(socket_path)
        # Receive a response from the server
        tsec_rec = client.recv(8)
        tsec_int = struct.unpack('q', tsec_rec)[0]
        tnsec_rec = client.recv(8)
        tnsec_int = struct.unpack('q', tnsec_rec)[0]
        rpm_rec = client.recv(4)
        rpm_float = struct.unpack('f', rpm_rec)[0]
        uk_rec = client.recv(4)
        uk_float = struct.unpack('f', uk_rec)[0]
        data = [tsec_int, tnsec_int, rpm_float, uk_float]
        if not(fifo.full()):	# if FIFO not full
            fifo.put(data)      # push data into FIFO
        else:                   # if FIFO full
            fifo.get(data)      # throw out one element
            fifo.put(data)      # push data into FIFO
        client.close()

# Function associated to thread4 (ZMQ publisher). This thread gets data from the FIFO queue and
# sends it to host computer using ZMQ publisher-subscriber paradigm.
def thread4_zmq_publisher(data):
    context = zmq.Context()
    zmq_socket = context.socket(zmq.PUB)
    zmq_socket.bind("tcp://*:5555")
    while True:
        a = 1
        if not(fifo.empty()):
            data = fifo.get()
            zmq_socket.send_string(','.join(map(str,data)))

# Main program (issues thread3 and thread4)
def main():
 
    # Create a pthread with default attributes and parameters (non-RT)
    thread_func_ptr3 = CFUNCTYPE(None, c_void_p)(thread3_client)
    ret = lc.pthread_create(byref(thread3), byref(attr), thread_func_ptr3, None)
    if ret !=0:
        print("create pthread failed\n")
        return ret

    # Create a pthread with default attributes and parameters (non-RT)
    thread_func_ptr4 = CFUNCTYPE(None, c_void_p)(thread4_zmq_publisher)
    ret = lc.pthread_create(byref(thread4), byref(attr), thread_func_ptr4, None)
    if ret !=0:
        print("create pthread failed\n")
        return ret

    # Join the thread and wait until it is done
    ret = lc.pthread_join(thread4, None)
    if ret !=0:
        print("join pthread failed: %m\n")
        return ret
    
    # Join the thread and wait until it is done
    ret = lc.pthread_join(thread3, None)
    if ret !=0:
        print("join pthread failed: %m\n")
        return ret
        
    return 0

print(main())
