#! /usr/bin/env python

"""

.. module:: bug_as
    :platform: Unix
    :synopsis: Python module for controlling the robot
    
.. moduleauthor:: Yassin Farah <s4801788@studenti.unige.it>

This node implements an action server to control the movement of the robot in the Gazebo environment

Subscribes to:
    | /scan
    | /odom
    
Publishes to:
    | /cmd_vel
   
Service:
    | /go_to_point_switch
    | /wall_follower_switch
    
Action server:
    | /reaching_goal
    
Parameter:
    | /des_pos_x
    | /des_pos_y

"""

import rospy
from geometry_msgs.msg import Point, Pose, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import actionlib
import actionlib.msg
import assignment_2_2022.msg
from tf import transformations
from std_srvs.srv import *
import time

srv_client_go_to_point_ = None
""" Service for moving the robot to a point
"""

srv_client_wall_follower_ = None
""" Service for making the robot follow a wall
"""

yaw_ = 0
""" Variable to store the value of the yaw angle
"""

yaw_error_allowed_ = 5 * (math.pi / 180)  # 5 degrees
""" Error for the yaw angle
"""

position_ = Point()
""" Message of type ``geometry_msgs::Twist`` for the position
"""

pose_ = Pose()
""" Message of type ``geometry_msgs::Pose`` for the pose
"""

desired_position_ = Point()
""" Message of type ``geometry_msgs::Twist`` to set the desired position of the ROS parameter
"""

desired_position_.z = 0

regions_ = None
""" Dictionary to store the regions of the laser
"""

state_desc_ = ['Go to point', 'wall following', 'done']
""" String descriptor of the state
"""

state_ = 0
""" Variable to store the state of the robot:
    0 - go to point
    1 - wall following
    2 - done
    3 - canceled
"""

# 0 - go to point
# 1 - wall following
# 2 - done
# 3 - canceled
# callbacks


def clbk_odom(msg):
    """Callback function to retrieve the position and the pose of the robot
    
    The function is called when a ``nav_msgs::Odometry`` message is published on the topic ``/odom``.
    It retrieves the *pose* and the *position* from the message and stores them in the global variables ``position_`` and ``pose_``.
    Finally, it computes the *yaw* angle from the pose, using quaternions.
    
    Args:
        msg: Message of type `nav_msgs::Odometry <http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html>`_
    """
    
    global position_, yaw_, pose_

    # position
    position_ = msg.pose.pose.position
    pose_ = msg.pose.pose

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def clbk_laser(msg):
    """Callback function to retrieve the laser output
    
    The function is called when a ``sensor_msgs::LaserScan`` message is published on the topic ``/scan``.
    It retrieves the laser output from the message, using the *ranges* parameter.
    
    Args:
        msg: Message of type `sensor_msgs::LaserScan <http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html>`_
    """
    
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }


def change_state(state):
    """Function to change the state of the robot
    
    The function changes the *state* of the robot and calls the appropriate *service*.
    If the state is 0, the ``/go_to_point_switch`` service is called, if the state is 1,
    the ``/wall_follower_switch`` service is called and if the state is 2, both services are cancelled.
    
    Args:
        state: Integer value representing the state to change to.
    """
    
    global state_, state_desc_
    global srv_client_wall_follower_, srv_client_go_to_point_
    state_ = state
    log = "state changed: %s" % state_desc_[state]
    rospy.loginfo(log)
    if state_ == 0:
        resp = srv_client_go_to_point_(True)
        resp = srv_client_wall_follower_(False)
    if state_ == 1:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)
    if state_ == 2:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(False)


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
    
def done():
    """Function to stop the robot
    
    The function publishes a null *velocity* as a ``geometry_msgs::Twist`` message to the ``/cmd_vel`` topic
    to stop the robot.
    """
    
    global pub
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)
    
    
def planning(goal):
    """Function to control the movement of the robot
    
    The function is called when a goal is set by the action client and it retrieves the *desired
    position* from the goal message. The ``/des_pos_x`` and ``/des_pos_y`` parameters are set and 
    the state of the robot is changed to 0. If the robot is in front of an obstacle, it changes state 
    to 1, until the obstacle is completely overtaken. Once the goal has been reached, the function 
    ``done()`` is called and the result is set to succeeded. In case the goal is cancelled, the function 
    ``done()`` is called.
    
    Args:
        goal: Goal set by the action client, containing the coordinates of the point to reach.
    """
    
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_, act_s, pose_
    change_state(0)
    rate = rospy.Rate(20)
    success = True
    
    desired_position_.x = goal.target_pose.pose.position.x
    desired_position_.y = goal.target_pose.pose.position.y
    rospy.set_param('des_pos_x', desired_position_.x)
    rospy.set_param('des_pos_y', desired_position_.y)
    
    
    feedback = assignment_2_2022.msg.PlanningFeedback()
    result = assignment_2_2022.msg.PlanningResult()
    
    while not rospy.is_shutdown():
        err_pos = math.sqrt(pow(desired_position_.y - position_.y, 2) +
                        pow(desired_position_.x - position_.x, 2))
        if act_s.is_preempt_requested():
            rospy.loginfo("Goal was preeempted")
            feedback.stat = "Target cancelled!"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            act_s.set_preempted()
            success = False
            change_state(2)
            done()
            break
        elif err_pos < 0.5:
            change_state(2)
            feedback.stat = "Target reached!"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            done()
            break       
        elif regions_ == None:
            continue
        
        elif state_ == 0:
            feedback.stat = "State 0: go to point"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            if regions_['front'] < 0.2:
                change_state(1)
        elif state_ == 1:
            feedback.stat = "State 1: avoid obstacle"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            desired_yaw = math.atan2(desired_position_.y - position_.y, desired_position_.x - position_.x)
            err_yaw = normalize_angle(desired_yaw - yaw_)
            if regions_['front'] > 1 and math.fabs(err_yaw) < 0.05:
                change_state(0)
        elif state_== 2:
            break
            
            
        else:
            rospy.logerr('Unknown state!')

        rate.sleep()
    
    if success:
        rospy.loginfo('Goal: Succeeded!')
        act_s.set_succeeded(result)
    
    

def main():
    """Main function
    
    The function is used to *initialize* the subscribers, the publisher, the services and 
    the action server. After the initialization, it waits for the client to set a new *goal*.
    """
    
    time.sleep(2)
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_, act_s, pub

    rospy.init_node('bug0')
    
    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    srv_client_go_to_point_ = rospy.ServiceProxy(
        '/go_to_point_switch', SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy(
        '/wall_follower_switch', SetBool)
    act_s = actionlib.SimpleActionServer('/reaching_goal', assignment_2_2022.msg.PlanningAction, planning, auto_start=False)
    act_s.start()
   
    # Wait for goal
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()
    
if __name__ == "__main__":
    main()
