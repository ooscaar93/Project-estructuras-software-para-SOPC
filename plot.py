# This program acts as ZMQ subscriber to the publisher defined in bridge.py
# (in the Rapsberry Pi). The program reads the data published by bridge.py
# (timestamp, RPM and uk) and plots it dinamically using pyqtgraph and PyQt5
# widget application.

# Imports
from PyQt5 import QtWidgets, QtCore     # Library to create Graphic Interfaces
import pyqtgraph as pg                  # Graphics and GUI library built on PyQt / PySide and numpy
from pyqtgraph import PlotWidget, plot
import sys                              # We need sys so that we can pass argv to QApplication
import os
import zmq                              # High performance communication library
import numpy as np

# The QMainWindow class provides a main application window
class MainWindow(QtWidgets.QMainWindow):
    
    # The constructor setups the main application window.
    # - Initializes two plots in the same window, one for motor RPM and other
    #   for uk (control signal).
    # - Reads first data published by bridge.py (publisher) and plots it.
    # - Setups timer for plot update and method to be executed in each update.
    # The dynamic plots have a time span of 100 seconds. Initially, the time
    # window that is shown corresponds to -100.0 to 0.0 seconds, and both
    # RPM and uk have 0 values assigned during this first time window.
    def __init__(self, *args, **kwargs):
        
        # Setup widget window
        super(MainWindow, self).__init__(*args, **kwargs)
        
        # Set window title
        self.setWindowTitle("Speed control of DC motor")
        
        # Create two plot widgets
        self.plot1 = pg.PlotWidget()
        self.plot2 = pg.PlotWidget()
        
        # Set background color for both plots
        self.plot1.setBackground('w')
        self.plot2.setBackground('w')
        
        # Set xlabel and ylabel
        self.plot1.setLabel(axis='bottom', text='t (secs)')
        self.plot1.setLabel(axis='left', text='RPM')
        self.plot2.setLabel(axis='bottom', text='t (secs)')
        self.plot2.setLabel(axis='left', text='uk (volts)')
        
        # Enable both x and y grid lines with an opacity of 0.5
        self.plot1.showGrid(x=True, y=True, alpha=0.5)
        self.plot2.showGrid(x=True, y=True, alpha=0.5)
        
        # Initialize data to plot for -100 seconds to 0 seconds with rpm and uk to 0
        self.t_list = np.arange(-100.0, 0.0, 0.1)            # 100 time points, with 0.1 sec separation
        self.rpm_list = np.zeros(len(self.t_list))           # 100 data points
        self.uk_list = np.zeros(len(self.t_list))            # 100 data points
        
        # Wait for first message from ZMQ publisher to store initial timestamp
        message = socket.recv_string()
        
        # Decode the received message
        parts = message.split(',')  # Split the message into parts (assuming comma-separated values)
        tsec = int(parts[0])        # Convert the first part to int (timestamp, secs)
        tnsec = int(parts[1])       # Convert the second part to int (timestamp, nsecs)
        rpm = float(parts[2])       # Convert the third part to float (RPM value)
        uk = float(parts[3])        # Convert the fourth part to float (uk value)
        
        # Calculate initial timestamp in seconds (float), and store it as absolute
        # reference for plotting (t0 corresponds to 0 secs in the graph)
        self.t0 = tsec + tnsec*1.0e-9
        
        # Add t0 data to data that will be ploted
        self.t_list = np.append(self.t_list, 0.0)
        self.rpm_list = np.append(self.rpm_list, rpm)
        self.uk_list = np.append(self.uk_list, uk)
        
        # Plot initial values
        pen1 = pg.mkPen(color=(255, 0, 0))
        pen2 = pg.mkPen(color=(0, 0, 255))
        self.data_line1 = self.plot1.plot(self.t_list, self.rpm_list, pen=pen1)
        self.data_line2 = self.plot2.plot(self.t_list, self.uk_list, pen=pen2)
        
        # Arrange the plots in a grid layout (RPM in the left, uk in the right)
        layout = QtWidgets.QGridLayout()
        layout.addWidget(self.plot1, 0, 0)
        layout.addWidget(self.plot2, 1, 0)
        
        # Setup and create central widget
        central_widget = QtWidgets.QWidget()
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)
        
        # Setup timer for plot update. update_plot_data will be executed once
        # every 50 milliseconds. This is considered enough time to plot all
        # data points acquired by the Raspberry Pi, since the used sampling
        # period is of 100 milliseconds, and still gives time to recover from
        # delays due to 100 - 50 = 50 millisecond slack.
        self.timer = QtCore.QTimer()
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()
    
    # This function waits to receive a new datapoint from ZMQ publisher, decodes
    # the received data, and updates plots by removing oldest datapoint and
    # appending newest datapoint.
    def update_plot_data(self):
        
        # Wait to receive new datapoint from ZMQ publisher
        message = socket.recv_string()
        
        # Decode the message
        parts = message.split(',')  # Split the message into parts (assuming comma-separated values)
        tsec = int(parts[0])        # Convert the first part to int (timestamp, secs)
        tnsec = int(parts[1])       # Convert the second part to int (timestamp, nsecs)
        rpm = float(parts[2])       # Convert the third part to float (RPM value)
        uk = float(parts[3])        # Convert the fourth part to float (uk value)
        
        # Calculate relative timestamp for plotting, using self.t0 as absolute reference
        t = tsec + tnsec*1.0e-9 - self.t0
        
        # Update plot values (remove oldest datapoint, add new datapoint)
        self.t_list = np.delete(self.t_list, 0)
        self.rpm_list = np.delete(self.rpm_list, 0)
        self.uk_list = np.delete(self.uk_list, 0)
        self.t_list = np.append(self.t_list, t)
        self.rpm_list = np.append(self.rpm_list, rpm)
        self.uk_list = np.append(self.uk_list, uk)
        
        # Update plot with new data
        self.data_line1.setData(self.t_list, self.rpm_list)
        self.data_line2.setData(self.t_list, self.uk_list)

# Create ZMQ subscriber, using 5555 port of raspberrypi-javi
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://raspberrypi-javi:5555")
socket.setsockopt_string(zmq.SUBSCRIBE, '')

# Create PyQt5 widget application
app = QtWidgets.QApplication(sys.argv)  # Create application
w = MainWindow()                        # Create main window according to MainWindow
w.show()                                # Show main window
sys.exit(app.exec_())                   # The exec() call starts the event-loop and will
                                        # block until the application quits. It is good practice to
                                        # pass on the exit code produced by exec() to sys.exit().