#! /usr/bin/env python

"""

.. module:: client
	:platform: Unix
	:synopsis: Python module for setting the robot goal position
    
.. moduleauthor:: Yassin Farah <s4801788@studenti.unige.it>

This node implements an action client to set the goal position of the robot and 
to publish the position and the velocity as a custom message.

Subscribes to:
	| /odom
    
Publishes to:
	| /pos_vel
    
Action server:
	| /reaching_goal

"""

import rospy
import math
import actionlib
import actionlib.msg
import assignment_2_2022.msg
from std_srvs.srv import *
import time
import sys
import select
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Twist
#import custom message
from assignment_2_2022.msg import Pos_vel

# Callback for the odometry subscriber
def clbk_odom(msg):
	"""Callback function to retrieve the position and the pose of the robot
    
	The function is called when a ``nav_msgs::Odometry`` message is published on the topic ``/odom``.
	It retrieves the *position* and the *linear velocity* from the message and publishes them as a custom message 
	on the topic ``/pos_vel``.
    
	Args:
		msg: Message of type `nav_msgs::Odometry <http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html>`_
	"""
	global pub
	# Get the position from the msg
	position_ = msg.pose.pose.position
	# Get the twist from the msg
	vel_lin = msg.twist.twist.linear
	# Create custom message
	pos_vel = Pos_vel()
	pos_vel.x = position_.x
	pos_vel.y = position_.y
	pos_vel.vel_x = vel_lin.x
	pos_vel.vel_y = vel_lin.y
	# Publish the custom message
	pub.publish(pos_vel)

# Function to extract numbers from string
def extract_numbers(input_str):
	"""Function to extract numbers from a string
    
	The function extracts numbers from a string with a defined format ("x,y") and returns them as a list.
    
	Args:
		input_str: String from which the function extracts the numbers
	
	Returns:
		A list of the float numbers in the string. If the casting went wrong, it returns None.
	"""
	# Separate the string in two
	nums = input_str.split(",")
	# Variables to store the extracted numbers
	num1 = 0.0
	num2 = 0.0
	# If input string is wrong format
	if len(nums) != 2:
		return
	# Convert the two strings to numbers
	try:
		num1 = float(nums[0])
	except ValueError:
		# If casting went wrong
		return
	try:
		num2 = float(nums[1])
	except ValueError:
		# If casting went wrong
		return
		
	return [num1, num2]

# Function to implement the action client
def client():
	"""Function that implements the action client
    
	The function implements the action client by initializing the client and 
	asking the user, through a command window, to input the desired position or 
	to cancel the previous goal. When the user inputs the desired position, the client 
	sends the goal to the action server. If the user cancels the request, the client 
	send a cancel request to the action server.

	"""
	
	# Create the action client
	client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2022.msg.PlanningAction)
	# Wait for the server to be started
	client.wait_for_server()
	
	# Flag to store if the goal was already canceled
	canceled = False
	# Message to print to the user
	message_string = "Insert the target position 'x,y' or insert 'c' to cancel it:"
	print(message_string)
	
	while not rospy.is_shutdown():
		# Listen for keyboard input in non-blocking mode
		input = select.select([sys.stdin], [], [], 1)[0]
		# If user entered some value
		if input:
			# Get the input string
			value = sys.stdin.readline().rstrip()
			value = value.replace(" ","")
 			
 			# If user entered 'c', cancel the goal
			if value == "c":
				if canceled:
					print("Goal not existing!")
				else:
					# Cancel the goal
					client.cancel_goal()
					canceled = True
			else:
				# Extract numbers from the string
				nums = extract_numbers(value)
				if nums is None:
					print("Input is in the wrong format!")
				else:
					# Create the goal to send to the server
					goal = assignment_2_2022.msg.PlanningGoal()
					goal.target_pose.pose.position.x = nums[0]
					goal.target_pose.pose.position.y = nums[1]
					
					# Send the goal to the action server
					client.send_goal(goal)
					canceled = False
			# Print again the message to the user
			print(message_string)
					

def main():
	"""Main function
	
	The function initializes the publisher and the subscriber and then calls the function ``client()``.
	"""
	
	global pub
	# Initialize the node
	rospy.init_node('action_client')
	# Publisher for the custom message
	pub = rospy.Publisher("/pos_vel", Pos_vel, queue_size=1)
	# Subscriber to retrieve position and speed
	sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
	# Start the action client
	client()
		
if __name__ == "__main__":
	main()	
