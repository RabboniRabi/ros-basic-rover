#!/usr/bin/env python

# Central node of the basic rover that keeps it roaming under the following directives:
# Keep moving forward if there are no obstacles
# Move right if there is an obstacle on the left
# Move left if there is an obstacle on the right
# Move back and then right if there is an obstacle on the front

from basic_rover.srv import ProximityCheck
from basic_rover.srv import ProximityCheckResponse
from basic_rover.msg import ObstacleInfo
from basic_rover.msg import DriveCommand

import rospy
import sys
import time

# Global variable: publisher - to avoid repeat initialisation
pub = rospy.Publisher('drive_rover', DriveCommand, queue_size = 10)

# The main function that calls other functions to get obstacle information, corresponding
# driving directions and publishing of driving directions
def roam():

	# Define a roamer node
	rospy.init_node('roamer', anonymous = True)

	while not rospy.is_shutdown():

		# Get the obstacle information
		obstacle_info = get_obstacle_info()
		rospy.loginfo("obstacle info: %s", obstacle_info.OBSTACLE_ON)

		# Drive the rover based on the obstacle information
		driving_dirs = get_driving_directions(obstacle_info)

		# Publish driving directions
		publish_driving_directions(driving_dirs)

		time.sleep(0.75)

# Get the obstacle information
def get_obstacle_info():

	# Get the service to check the proximity
	rospy.wait_for_service('proximity_service')

	try:
		# Check the proximity of obstacles
		proximity_service = rospy.ServiceProxy('proximity_service', ProximityCheck)
		proximity_info = proximity_service()
		return proximity_info.obstacle_info
	except KeyboardInterrupt:
		rospy.loginfo("Exiting roaming node")
		sys.exit(1)
	except rospy.ServiceException, e:
		print("Proximity service exception: %s" %e)


# Decide the driving directions based on obstacle information
def get_driving_directions(obstacle_info):

	# A list to hold the sequeunce of driving directions
	driving_list = list()

	if (obstacle_info.OBSTACLE_ON == ObstacleInfo.CLEAR):

		# No Obstacle, drive ahead two steps
		drive_step_1 = DriveCommand()
		drive_step_1.DIRECTION = DriveCommand.FORWARD

		drive_step_2 = DriveCommand()
		drive_step_2.DIRECTION = DriveCommand.FORWARD

		driving_list.append(drive_step_1)
		driving_list.append(drive_step_2)


	elif (obstacle_info.OBSTACLE_ON == ObstacleInfo.LEFT):

		# Drive right
		drive_step = DriveCommand()
		drive_step.DIRECTION = DriveCommand.RIGHT

		driving_list.append(drive_step)

	elif (obstacle_info.OBSTACLE_ON == ObstacleInfo.RIGHT):

		# Drive left
		drive_step = DriveCommand()
		drive_step.DIRECTION = DriveCommand.LEFT

		driving_list.append(drive_step)

	elif (obstacle_info.OBSTACLE_ON == ObstacleInfo.FRONT):

		# Drive back and then right when an obstacle is in front

		drive_step_1 = DriveCommand()
		drive_step_2 = DriveCommand()

		drive_step_1.DIRECTION = DriveCommand.BACK
		drive_step_2.DIRECTION = DriveCommand.RIGHT

		driving_list.append(drive_step_1)
		driving_list.append(drive_step_2)

	return driving_list

# Publishes given list of driving directions
def publish_driving_directions(list):

	# Iterate and publish the driving directions
	for direction in list:
		 pub.publish(direction)


if __name__ == "__main__":
	roam()

