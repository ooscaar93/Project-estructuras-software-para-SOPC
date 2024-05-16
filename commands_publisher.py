#!/usr/bin/env

# This program waits for setpoint to be introduced by user and sends it to
# raspberry py via ZMQ (this program acts as publisher).

import zmq

# Main function
def publisher():
    
    # Create ZMQ publisher
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:5556")  # Bind to port 5556, for communication with raspberry pi

    while True:
        try:
            # Get a float value for the new setpoint from user
            sp = float(input("Please insert new setpoint (between -150.0 and 150.0 RPM): "))
            # If setpoint is out of range, publisher doesn't send it
            if (sp > 150.0 or sp < -150.0):
                print("Setpoint out of range.")
            else:
                # Send the float value as bytes over ZeroMQ socket
                socket.send_string(f"{sp}")
                print(f"Sent: {sp}")
        # Except block executes if introduced value cannot be converted to float
        except ValueError:
            print("Invalid input, try again.")

# Run main program
if __name__ == "__main__":
    publisher()