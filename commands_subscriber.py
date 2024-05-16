#!/usr/bin/env

# This program (in the raspberry pi) acts as ZMQ subscriber and waits for setpoint
# to be sent by commands_publisher.py.
# This program (in the raspberry pi) acts as ZMQ subscriber to commands_publisher.py
# program (ZMQ publisher, in PC). The publisher sends the setpoint issued by the user
# from the computer, and the subscriber (this program) receives it.
# This program also acts as UNIX socket client, to send the setpoint to control_motor,
# which acts as UNIS socket server.

from ctypes import *
import zmq
import socket
import os
import struct

# Main function
def subscriber():
    
    # Create ZMQ publisher
    context = zmq.Context()
    socket_zmq = context.socket(zmq.SUB)
    socket_zmq.connect("tcp://169.254.1.2:5556")  # Connect to publisher's address
    # This IP address corresponds to ethernet connection between raspberry and PC.
    # This address should be modified in PC or in raspberry according to the
    # address used in each device.

    # Subscribe to all topics (empty prefix)
    socket_zmq.setsockopt_string(zmq.SUBSCRIBE, "")
    
    # Set the path for the Unix socket
    socket_path = '/tmp/setpoint_socket'
    
    while True:

        # Receive the message (new setpoint) from publisher
        message = socket_zmq.recv_string()
        
        # Decode the received message as a float
        try:
            sp = float(message)
            print(f"Received: {sp}")
        except ValueError:
            print("Received invalid data.")
        
        # Create the Unix socket client
        client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        # Connect to the server (in control_motor)
        client.connect(socket_path)
        # Send message to server (in control_motor)
        client.sendall(struct.pack('f', sp))
        client.close()

# Run main function
if __name__ == "__main__":
    subscriber()
