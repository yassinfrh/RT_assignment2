#! /usr/bin/env python

"""

.. module:: go_to_point_service
    :platform: Unix
    :synopsis: Python module to make the robot reach a point
    
.. moduleauthor:: Yassin Farah <s4801788@studenti.unige.it>

This node implements a service to make the robot move towards a point in the arena.

Subscribes to:
    | /odom
    
Publishes to:
    | /cmd_vel
   
Service:
    | /go_to_point_switch
    
Parameter:
    | /des_pos_x
    | /des_pos_y

"""

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import time

import math

active_ = False
""" Boolean to check if the service is active or not

    :meta hide-value:
"""

# robot state variables
position_ = Point()
""" Message of type ``geometry_msgs::Point`` for the position

    :meta hide-value:
"""

yaw_ = 0
""" Variable to store the value of the yaw angle

    :meta hide-value:
"""

# machine state
state_ = 0
""" | Variable to store the state of the robot:
    | 0 - turn
    | 1 - go straight
    | 2 - done
    
    :meta hide-value:
"""

# goal
desired_position_ = Point()
""" Message of type ``geometry_msgs::Point`` to set the desired position of the ROS parameter

    :meta hide-value:
"""

desired_position_.x = rospy.get_param('des_pos_x')
desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.z = 0
# parameters
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.3

kp_a = 3.0  # In ROS Noetic, it may be necessary to change the sign of this proportional controller
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6

# publishers
pub = None
""" Publisher to the topic ``/cmd_vel``

    :meta hide-value:
"""

# service callbacks


def go_to_point_switch(req):
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

# callbacks


def clbk_odom(msg):
    """Callback function to retrieve the position and the pose of the robot
    
    The function is called when a ``nav_msgs::Odometry`` message is published on the topic ``/odom``.
    It retrieves the *pose* and the *position* from the message and stores them in the global variables ``position_`` and ``pose_``.
    Finally, it computes the *yaw* angle from the pose, using quaternions.
    
    Args:
        msg: Message of type `nav_msgs::Odometry <http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html>`_
    """
    
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def change_state(state):
    """Function to change the state
    
    Args:
        state: Integer value representing the state to change to.
    """
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)


def normalize_angle(angle):
    """Function to normalize an angle
    
    The function normalizes the *angle* given in input using the ``math`` library
    
    Args:
        angle: Float number representing the angle to normalize
        
    Returns:
        The normalized angle
    """
    
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def fix_yaw(des_pos):
    """Function to turn the robot
    
    The function computes the desired yaw angle and publishes the appropriate 
    angular velocity to the topic ``/cmd_vel``. Once the desired yaw is reached, 
    the function changes the state to 1.
    
    Args:
        des_pos: Message of type `geometry_msgs::Point <http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Point.html>`_
    """
    
    global yaw_, pub, yaw_precision_2_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)

    rospy.loginfo(err_yaw)

    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a

    pub.publish(twist_msg)

    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        print ('Yaw error: [%s]' % err_yaw)
        change_state(1)


def go_straight_ahead(des_pos):
    """Function to move the robot straight
    
    The function makes the robot move straight to the desired position by publishing the appropriate linear 
    velocity to the topic ``/cmd_vel``. If the yaw angle is wrong, the function changes the state to 0. 
    If the position is reached, the function changes the state to 2.
    
    Args:
        des_pos: Message of type `geometry_msgs::Point <http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Point.html>`_
    """
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = kp_d*(err_pos)
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a*err_yaw
        pub.publish(twist_msg)
    else:
        print ('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        print ('Yaw error: [%s]' % err_yaw)
        change_state(0)


def done():
    """Function to stop the robot
    
    The function publishes a null *velocity* as a ``geometry_msgs::Twist`` message to the ``/cmd_vel`` topic
    to stop the robot.
    """
    
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)
                

def main():
    """Main function
    
    The function is used to *initialize* the subscriber, the publisher and the service. 
    After the initialization, if the ``active_`` variable is set to true, the function 
    checks the ``state`` variable and calls the appropriate function.
    """
    
    global pub, active_

    rospy.init_node('go_to_point')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)

    srv = rospy.Service('go_to_point_switch', SetBool, go_to_point_switch)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            continue
        else:
            desired_position_.x = rospy.get_param('des_pos_x')
            desired_position_.y = rospy.get_param('des_pos_y')
            if state_ == 0:
                fix_yaw(desired_position_)
            elif state_ == 1:
                go_straight_ahead(desired_position_)
            elif state_ == 2:
                done()
            else:
                rospy.logerr('Unknown state!')

        rate.sleep()


if __name__ == '__main__':
    main()
