#!/usr/bin/env python

# Commander node of the basic rover that recieves keyboard input and publishes driving directions

from basic_rover.srv import KeyCommand
from basic_rover.srv import KeyCommandResponse
from basic_rover.msg import DriveCommand
import rospy

# Global variable: publisher - To avoid repeat initialisation
pub = rospy.Publisher('drive_rover', DriveCommand, queue_size = 10)


def commander_service():
	# Initialise the node
	rospy.init_node('rover_commander', anonymous = True)
	# Declare the service
	service = rospy.Service('rover_commander', KeyCommand, handle_input)
	print("Commander Serice is up, Sir!")
	rospy.spin()

# The function that on invocation of the commander service gets the driving
# directions from the input request parameters and makes a call to publish
# the driving commands.
def handle_input(req):

	# Get the driving directions from the keyboard input request
	drive = get_direction(req)

	# Publish driving command
	if (drive.DIRECTION != DriveCommand.INVALID):
		publish_drive_command(drive)

	# Return empty reponse
	return KeyCommandResponse()


# Extract the driving direction from the input
def get_direction(input):

	drive = DriveCommand()
	if (input.twist.linear.x > 0):
		drive.DIRECTION = DriveCommand.FORWARD
	elif (input.twist.linear.x < 0):
		drive.DIRECTION = DriveCommand.BACK
	elif (input.twist.angular.z > 0):
		drive.DIRECTION = DriveCommand.LEFT
	elif (input.twist.angular.z < 0):
		drive.DIRECTION = DriveCommand.RIGHT
	else:
		rospy.loginfo("Invalid driving command")
		drive.DIRECTION = DriveCommand.INVALID
	return drive

# Publish the driving direction
def publish_drive_command(drive_command):
	rospy.loginfo("going to publish the drive command")
	pub.publish(drive_command)


if __name__ == "__main__":
	commander_service()
