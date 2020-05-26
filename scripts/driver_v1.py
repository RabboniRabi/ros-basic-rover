#!/usr/bin/env python

from basic_rover.msg import DriveCommand
import RPi.GPIO as GPIO
import time
import rospy


def driving_command_listener():
	# Initialise the node
	rospy.init_node('rover', anonymous = True)
	# Subscribe to the commander topic
	rospy.Subscriber("drive_rover", DriveCommand, driver)
	rospy.spin()


def driver(data):
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
	time.sleep(1)
	GPIO.output(13, False)
	GPIO.output(19, False)

def drive_back():
	# Drive backward for a second
	rospy.loginfo("Going back. *Beep!* *Beep!*")
	GPIO.output(11, True)
	GPIO.output(15, True)
	time.sleep(1)
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
	time.sleep(1)
	GPIO.output(13, False)

# Set up the GPIO pins on the RaspberryPi
def initialise_pins():
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(11, GPIO.OUT)
	GPIO.setup(13, GPIO.OUT)
	GPIO.setup(15, GPIO.OUT)
	GPIO.setup(19, GPIO.OUT)


if __name__ == "__main__":
	try:
		initialise_pins()
		driving_command_listener()
	except KeyboardInterrupt:
		print("Doing cleanup")
		GPIO.cleanup()
