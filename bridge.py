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
	tsec_rec = client.recv(8)
	tsec_int = struct.unpack('q', tsec_rec)[0]
	tnsec_rec = client.recv(8)
	tnsec_int = struct.unpack('q', tnsec_rec)[0]
	rpm_rec = client.recv(4)
	rpm_float = struct.unpack('f', rpm_rec)[0]
	uk_rec = client.recv(4)
	uk_float = struct.unpack('f', uk_rec)[0]
	print(tsec_int)
	print(tnsec_int)
	print(f'RPM: ', rpm_float)
	print(f'uk : ', uk_float)
	# Close the connection
	client.close()
