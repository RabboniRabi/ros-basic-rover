#!/usr/bin/env python

# Version 2 of the driving node for the basic rover.
# In this, the script will be able to accept upto 5 successive driving commands
# that could be sent without waiting for each driving command to execute.
# The driving commands are stored in a buffer and executed by order of arrival

from basic_rover.msg import DriveCommand
from Queue import Queue
from threading import Thread

import RPi.GPIO as GPIO
import time
import rospy

# Initialise a queue to hold driving commands  of size 5
drive_command_queue = Queue(maxsize = 5)

# The time for which the motors will be spinning
driving_time = 1

def driving_command_listener():
	# Initialise the node
	rospy.init_node('rover', anonymous = True)
	# Subscribe to the topic publishing driving commands
	rospy.Subscriber("drive_rover", DriveCommand, command_handler)
	rospy.spin()

# Function to add published commands received to the queue
def command_handler(drive_command):
	if not drive_command_queue.full():
		rospy.loginfo("In queue: %s", drive_command.DIRECTION)
		drive_command_queue.put(drive_command)

# Function to keep checking the queue and execute drive commands
def poll_and_exec_drive_commands():
	while not rospy.is_shutdown():
		if not drive_command_queue.empty():
			drive(drive_command_queue.get())

# Function to call the appropriate direction based driving function
def drive(data):
	if (data.DIRECTION == DriveCommand.FORWARD):
		drive_forward()
	elif (data.DIRECTION == DriveCommand.BACK):
		drive_back()
	elif (data.DIRECTION == DriveCommand.LEFT):
		drive_left()
	elif (data.DIRECTION == DriveCommand.RIGHT):
		drive_right()
	else :
		rospy.loginfo("No valid direction given. Not going to drive!")

def drive_forward():
	# Drive forward for a second
	rospy.loginfo("Going forward!")
	GPIO.output(13, True)
	GPIO.output(19, True)
	time.sleep(driving_time)
	GPIO.output(13, False)
	GPIO.output(19, False)

def drive_back():
	# Drive backward for a second
	rospy.loginfo("Going back *beep!* *beep!*")
	GPIO.output(11, True)
	GPIO.output(15, True)
	time.sleep(driving_time)
	GPIO.output(11, False)
	GPIO.output(15, False)

def drive_left():
	# Drive left for a second
	rospy.loginfo("Going left!")
	GPIO.output(19, True)
	time.sleep(1)
	GPIO.output(19, False)

def drive_right():
	# Drive right for a second
	rospy.loginfo("Going right!")
	GPIO.output(13, True)
	time.sleep(driving_time)
	GPIO.output(13, False)

# Set up the GPIO pins on the RaspberryPi
def initialise_pins():
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(11, GPIO.OUT)
	GPIO.setup(13, GPIO.OUT)
	GPIO.setup(15, GPIO.OUT)
	GPIO.setup(19, GPIO.OUT)

# Start a separate thread for function that polls the driving command queue
def start_drive_queue_polling_thread():
	drive_queue_poller = Thread(target=poll_and_exec_drive_commands)
	drive_queue_poller.start()

if __name__ == "__main__":
	try:
		initialise_pins()
		start_drive_queue_polling_thread()
		driving_command_listener()
	except KeyboardInterrupt:
		print("Doing cleanup")
		GPIO.cleanup()
