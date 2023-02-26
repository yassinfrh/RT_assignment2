#! /usr/bin/env python

"""

.. module:: wall_follow_service
    :platform: Unix
    :synopsis: Python module to make the robot follow a wall
    
.. moduleauthor:: Yassin Farah <s4801788@studenti.unige.it>

This node implements a service to make the robot follow a wall, not to bump into it.

Subscribes to:
    | /scan
    
Publishes to:
    | /cmd_vel
   
Service:
    | /wall_follower_switch

"""

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math

active_ = False
""" Boolean to check if the service is active or not

    :meta hide-value:
"""

pub_ = None
""" Publisher to the topic ``/cmd_vel``

    :meta hide-value:
"""

regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}
""" Dictionary to store the regions of the laser

    :meta hide-value:
"""

state_ = 0
""" | Variable to store the state of the robot:
    | 0 - find wall
    | 1 - turn left
    | 2 - follow the wall
    
    :meta hide-value:
"""

state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}
""" Dictionary to store the description of each state

    :meta hide-value:
"""


def wall_follower_switch(req):
    """Callback function for the service
    
    The function is called when the service is called. It sets the variable ``active_`` to true 
    and returns reply message
    
    Args:
        req: Request message
        
    Returns:
        The reply message
    """
    
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res


def clbk_laser(msg):
    """Callback function to retrieve the laser output
    
    The function is called when a ``sensor_msgs::LaserScan`` message is published on the topic ``/scan``.
    It retrieves the laser output from the message, using the *ranges* parameter.
    Finally, it calls the function ``take_action()``.
    
    Args:
        msg: Message of type `sensor_msgs::LaserScan <http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html>`_
    """
    
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:713]), 10),
    }

    take_action()


def change_state(state):
    """Function to change the state
    
    Args:
        state: Integer value representing the state to change to.
    """
    
    global state_, state_dict_
    if state is not state_:
        print ('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state


def take_action():
    """Function to control the laser output
    
    The function checks which regions of the laser detected and obstacle and 
    it changes the state appropriately.
    """
    
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    state_description = ''

    d0 = 1
    d = 1.5

    if regions['front'] > d0 and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d0 and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d0 and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 3 - fright'
        change_state(2)
    elif regions['front'] > d0 and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 4 - fleft'
        change_state(0)
    elif regions['front'] < d0 and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 5 - front and fright'
        change_state(1)
    elif regions['front'] < d0 and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 6 - front and fleft'
        change_state(1)
    elif regions['front'] < d0 and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    elif regions['front'] > d0 and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 8 - fleft and fright'
        change_state(0)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)


def find_wall():
    """Function to find the wall
    
    The function creates a message of type ``geometry_msgs::Twist``, sets the appropriate 
    linear and angular velocities and returns the message.
    
    Returns:
        The message of type ``geometry_msgs::Twist``.
    """
    
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = -0.3
    return msg


def turn_left():
    """Function to turn the robot left
    
    The function creates a message of type ``geometry_msgs::Twist``, sets the appropriate 
    angular velocity and returns the message.
    
    Returns:
        The message of type ``geometry_msgs::Twist``.
    """
    
    msg = Twist()
    msg.angular.z = 0.3
    return msg


def follow_the_wall():
    """Function to follow the wall
    
    The function creates a message of type ``geometry_msgs::Twist``, sets the appropriate 
    linear velocity and returns the message.
    
    Returns:
        The message of type ``geometry_msgs::Twist``.
    """
    
    global regions_

    msg = Twist()
    msg.linear.x = 0.5
    return msg


def main():
    """Main function.
    
    The function initializes the publisher, the subscriber and the service. 
    Then, if the variable ``active_`` is set to true, it checks the state 
    and calls the appropriate function.
    """
    
    global pub_, active_

    rospy.init_node('reading_laser')

    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)

    srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            rate.sleep()
            continue
        else:
            msg = Twist()
            if state_ == 0:
                msg = find_wall()
            elif state_ == 1:
                msg = turn_left()
            elif state_ == 2:
                msg = follow_the_wall()
            else:
                rospy.logerr('Unknown state!')

            pub_.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    main()
