#!/usr/bin/env python3

import socket
import struct
import os

# Set the path for the Unix socket
socket_path = './control_socket'

while True:
	# Create the Unix socket client
	client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
	# Connect to the server
	client.connect(socket_path)
	# Receive a response from the server
	response = client.recv(1024)
	res = struct.unpack('f',response)[0]
	print(f'Received response:',res)
	# Close the connection
	client.close()
