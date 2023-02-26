#! /usr/bin/env python

"""

.. module:: info_node
	:platform: Unix
	:synopsis: Python module printing the robot's info
    
.. moduleauthor:: Yassin Farah <s4801788@studenti.unige.it>

This node prints with a frequency set through a ROS parameter in the launch file 
the distance from the desired position and the average speed of the robot.

Subscribes to:
	| /pos_vel
    
Parameter:
	| /publish_frequency
	| /des_pos_x
	| /des_pos_y

"""

import rospy
import math
import time
#import custom message
from assignment_2_2022.msg import Pos_vel

# Frequency with which the info is printed (1 is just default value)
freq = 1.0
""" Float variable for the printing frequency

	:meta hide-value:
"""

# Last time the info was printed
last_printed = 0
""" Integer variable for storing the last instant in which the info was printed

	:meta hide-value:
"""

# Callback function for the info subscriber
def clbk_posvel(msg):
	"""Callback function to retrieve the position and the velocity of the robot
	
	The function is called when a message is published on the topic ``/pos_vel``. 
	It retrieves the desired position from the ROS parameters and the robot's position 
	from the message to compute the distance from the desired position. 
	It retrieves the robot's velocity to compute the average speed. It finally prints 
	the info, with a certain frequency, on the command window.
	"""
	
	global freq, last_printed
	# If frequency parameter is bad, set the frequency to default
	if freq <= 0.0:
		freq = 1.0
	# Time period in milliseconds
	t_period = (1.0/freq) * 1000
	# Get current time in milliseconds
	curr_t = time.time() * 1000
	# If enough time passed since last_printed, print the info
	if curr_t - last_printed > t_period:
		# Retrieve the desired position
		des_x = rospy.get_param("des_pos_x")
		des_y = rospy.get_param("des_pos_y")
		# Retrieve the actual position
		x = msg.x
		y = msg.y
		# Compute the distance
		dist = math.dist([des_x, des_y], [x, y])
		# Compute the average speed
		avg_vel = math.sqrt(msg.vel_x**2 + msg.vel_y**2)
		# Prepare the message to print
		dist_str = "Distance from the desired position: " + str(dist)
		vel_str = "Average speed: " + str(avg_vel)
		# Print the info
		print(dist_str)
		print(vel_str)
		print()
		# Update last_printed
		last_printed = curr_t
	

def main():
	"""Main function
	
	The function initializes the subscriber and retrieves the printing frequency 
	from the ROS parameter and it waits.
	"""
	
	global freq
	# Initialize the node
	rospy.init_node('info_node')
	# Get the publish frequency parameter
	freq = rospy.get_param("publish_frequency")
	# Subscriber for the custom message
	sub_odom = rospy.Subscriber("/pos_vel", Pos_vel, clbk_posvel)
	
	# Wait
	rospy.spin()
	
if __name__ == "__main__":
	main()	
