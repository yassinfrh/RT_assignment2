#! /usr/bin/env python

"""

.. module:: goals_service
	:platform: Unix
	:synopsis: Python module for printing the number of goals reached
    
.. moduleauthor:: Yassin Farah <s4801788@studenti.unige.it>

This node implements a service that, when called, prints the number of goals reached 
and the number of goals cancelled.

Subscribes to:
	| reaching_goal/result
    
Service:
	| /goals

"""

import rospy
from assignment_2_2022.srv import Goals_rc, Goals_rcResponse
import actionlib
import actionlib.msg
import assignment_2_2022.msg

# Variables to store the number of times goal was cancelled or reached
canc_n = 0;
""" Integer variable that stores the number of cancelled goals.

	:meta hide-value:
"""

reach_n = 0;
""" Integer variable that stores the number of reached goals.

	:meta hide-value:
"""

# Callback for result subscriber
def clbk_result(msg):
	"""Callback function to retrieve the status of the goal
    
	The function is called when a result message is published on the topic ``reaching_goal/result`` 
	and it checks the status of the result: if it's 2, it increases the number of cancelled goals, 
	if it's 3, it increases the number of reached goals.
    
	Args:
		msg: Message of type ``assignment_2_2022::PlanningActionResult``
	"""
	
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
	"""Callback function for the service
	
	The function is called when the service is called and it simply returns the message 
	containing the variables ``reach_n`` and ``canc_n``.
	"""
	
	global canc_n, reach_n
	
	# Return the response
	return Goals_rcResponse(reach_n, canc_n)

def main():
	"""Main function
	
	The function initializes the service and the subscriber and waits.
	"""
	
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
