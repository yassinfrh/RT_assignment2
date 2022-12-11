#! /usr/bin/env python

import rospy
from assignment_2_2022.srv import Goals_rc, Goals_rcResponse
import actionlib
import actionlib.msg
import assignment_2_2022.msg

# Variables to store the number of times goal was cancelled or reached
canc_n = 0;
reach_n = 0;

# Callback for result subscriber
def clbk_result(msg):
	global canc_n, reach_n
	# Get the status of the result
	status = msg.status.status
	# If status is 2, the goal was preempted
	if status == 2:
		canc_n+= 1
	# If status is 3, the goal was reached
	elif status == 3:
		reach_n+= 1
		
# Service function
def get_goals(req):
	global canc_n, reach_n
	
	# Return the response
	return Goals_rcResponse(reach_n, canc_n)

def main():
	# Initialize the node
	rospy.init_node('goals_service')
	# Create the service
	srv = s = rospy.Service('goals', Goals_rc, get_goals)
	# Subscribe to the result topic
	sub_result = rospy.Subscriber('/reaching_goal/result', assignment_2_2022.msg.PlanningActionResult, clbk_result)
	
	# Wait
	rospy.spin()
	
if __name__ == "__main__":
    main()
