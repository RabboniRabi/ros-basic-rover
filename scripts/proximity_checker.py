#!/usr/bin/env python

# Node that checks for the proximity of objects using two ultrasonic sensors (HC-SR04)
# connected to the front of the rover.
# This script contains a service definition that can be called by
# any other node to determine obstacle information


from basic_rover.msg import ObstacleInfo
from basic_rover.srv import ProximityCheck
from basic_rover.srv import ProximityCheckResponse
from threading import Thread
from threading import Lock

import rospy
import time
import RPi.GPIO as GPIO


# Global pin number declarations
LEFT_SENSE_TRIG = 16
LEFT_SENSE_ECHO = 18

RIGHT_SENSE_TRIG = 22
RIGHT_SENSE_ECHO = 24

# The distance used to check for obstacle proximity in centimeters
PROXIMITY = 20
# The time specified for the trigger pin of HC-SR04 sensor to the be high
TRIGGER_TIME = 0.00001

# Global obstacle flags
is_obstacle_on_left = False
is_obstacle_on_right = False

# Function to initialise this node and declare the service
def proximity_service():
	# Initialise the node
	rospy.init_node('proximity_service', anonymous = True)
	# Declare the service
	service = rospy.Service('proximity_service', ProximityCheck, check_proximity)
	# Keep the service running
	rospy.spin()

# Handler which is invoked when proximity service is called
def check_proximity(req):

	# Check the left sensor
	left_sensor_thread = Thread(target = check_left_sensor)
	left_sensor_thread.start()

	# Give a small gap between thread execution
	time.sleep(0.25)

	# Check the right sensor
	right_sensor_thread = Thread(target = check_right_sensor)
	right_sensor_thread.start()

	left_sensor_thread.join()
	right_sensor_thread.join()

	obstacle = ObstacleInfo()

	if (is_obstacle_on_left and is_obstacle_on_right):
		obstacle.OBSTACLE_ON = obstacle.FRONT
		rospy.loginfo("obstacle on front")
	elif (is_obstacle_on_left):
		obstacle.OBSTACLE_ON = obstacle.LEFT
		rospy.loginfo("obstacle on left")
	elif (is_obstacle_on_right):
		obstacle.OBSTACLE_ON = obstacle.RIGHT
		rospy.loginfo("obstacle on right")
	else:
		obstacle.OBSTACLE_ON = obstacle.CLEAR
		rospy.loginfo("clear! no obstacles")

	return ProximityCheckResponse(obstacle)

# Checks the distance of any obstacle on the left.
# Sets the global left obstacle flag to true if distance
# is less than the proximity distance
def check_left_sensor():

	GPIO.output(LEFT_SENSE_TRIG, False)
	time.sleep(1)  # wait for sensor to settle
	# Trigger the echo
	GPIO.output(LEFT_SENSE_TRIG, True)
	time.sleep(TRIGGER_TIME)
	GPIO.output(LEFT_SENSE_TRIG, False)

	# Measure the echo pulse duration
	while GPIO.input(LEFT_SENSE_ECHO) == 0:
		pulse_start = time.time()
	while GPIO.input(LEFT_SENSE_ECHO) == 1:
		pulse_end = time.time()

	pulse_duration = pulse_end - pulse_start

	distance = pulse_duration * (17150)
	distance = round(distance, 2)

	rospy.loginfo("Distance to obstacle on left sensor: %s", distance)

	# If obstacle is in proximity, set the left sensor flag to true
	global is_obstacle_on_left
	if distance <= PROXIMITY:
		is_obstacle_on_left = True
	else:
		is_obstacle_on_left = False

# Checks the distance of any obstacle on the right.
# Sets the global right obstacle flag to true if distance
# is less than the proximity distance
def check_right_sensor():
	GPIO.output(RIGHT_SENSE_TRIG, False)
	time.sleep(1)  # wait for sensor to settle
	# Trigger the echo
	GPIO.output(RIGHT_SENSE_TRIG, True)
	time.sleep(TRIGGER_TIME) 
	GPIO.output(RIGHT_SENSE_TRIG, False)

	# Measure the echo pulse duration
	while GPIO.input(RIGHT_SENSE_ECHO) == 0:
		pulse_start = time.time()

	while GPIO.input(RIGHT_SENSE_ECHO) == 1:
		pulse_end = time.time()

	pulse_duration = pulse_end - pulse_start

	distance = pulse_duration * (17150)
	distance = round(distance, 2)

	rospy.loginfo("Distance to obstacle on right sensor: %s", distance)

	# If obstacle is in proximity, set the right sensor flag to true
	global is_obstacle_on_right
	if distance <= PROXIMITY:
		is_obstacle_on_right = True
	else:
		is_obstacle_on_right = False



def initialise_sensors():
	GPIO.cleanup()
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(LEFT_SENSE_TRIG, GPIO.OUT)
	GPIO.setup(LEFT_SENSE_ECHO, GPIO.IN)
	GPIO.setup(RIGHT_SENSE_TRIG, GPIO.OUT)
	GPIO.setup(RIGHT_SENSE_ECHO, GPIO.IN)


if __name__ == "__main__":
	try:
		initialise_sensors()
		proximity_service()
	except KeyboardInterrupt:
		print "Cleaning up the pins"
		GPIO.cleanup()
