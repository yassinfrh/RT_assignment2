#! /usr/bin/env python

import rospy
import math
import time
#import custom message
from assignment_2_2022.msg import Pos_vel

# Frequency with which the info is printed (1 is just default value)
freq = 1.0
# Last time the info was printed
last_printed = 0

# Callback function for the info subscriber
def clbk_posvel(msg):
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
