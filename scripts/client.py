#! /usr/bin/env python

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
