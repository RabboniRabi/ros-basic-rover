#!/usr/bin/env python

# Node that listens to keyboard input and passes on the value in a service call to the commander node

import rospy
from geometry_msgs.msg import Twist
from basic_rover.srv import KeyCommand

# Call back function that calls the commander service with the kyeboard data
def callback(data):

	if(data.linear.x != 0 or data.angular.z != 0) :
		rospy.loginfo("Calling the rover commander with the values")
		rospy.wait_for_service('rover_commander')
		try:
			commander_service = rospy.ServiceProxy('rover_commander', KeyCommand)
			commander_service(data)
		except rospy.ServiceException, e:
			print("Commander service exception: %s" %e)

# Function to initialise the node and subscribe to the keyboard topic
def listener():

	# Initialise the node
	rospy.init_node('listener', anonymous = True)
	# Subscribe to the topic the keyboard commands will be published on
	rospy.Subscriber("key_vel", Twist, callback)
	# Keep the script from exiting until the node has stopped
	rospy.spin()

if __name__ == '__main__':
	listener()
