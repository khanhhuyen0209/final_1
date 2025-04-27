#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from nav_msgs.msg import OccupancyGrid  # Add this import
from sensor_msgs.msg import LaserScan
import tf
from math import pi, atan2, sqrt

class FrontierExplorer:
    def __init__(self):
        rospy.init_node('frontier_explorer', anonymous=True)

        # Publishers and Subscribers
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        # Some internal state
        self.map = None
        self.robot_position = None

    def map_callback(self, msg):
        # Store the map
        self.map = msg

    def laser_callback(self, msg):
        # Optionally use laser scan data for obstacle detection
        self.laser_data = msg

    def get_robot_position(self):
        # Get the current robot's position using tf
        listener = tf.TransformListener()
        try:
            (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            return trans[0], trans[1]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None

    def explore_frontiers(self):
        if self.map is None:
            rospy.loginfo("Waiting for map...")
            return

        # Here you would implement the frontier detection algorithm
        # For now, we'll just go to a random location for exploration
        frontier_goal = PoseStamped()
        frontier_goal.header.frame_id = "map"
        frontier_goal.pose.position.x = 5.0  # Just an example
        frontier_goal.pose.position.y = 5.0  # Just an example
        frontier_goal.pose.orientation.w = 1.0

        # Publish the goal
        self.goal_pub.publish(frontier_goal)
        rospy.loginfo("Sending frontier goal!")

        # Control the robot to move to the goal
        self.move_to_goal(frontier_goal)

    def move_to_goal(self, goal):
        # A simple move_to_goal method that uses cmd_vel for basic movement
        twist = Twist()
        # Move to goal logic, you can improve this with simple PID or a controller
        while not rospy.is_shutdown():
            current_pos = self.get_robot_position()
            if current_pos is None:
                rospy.logwarn("Could not get robot position!")
                continue

            # Check if the robot is close enough to the goal
            distance = sqrt((goal.pose.position.x - current_pos[0])**2 + (goal.pose.position.y - current_pos[1])**2)
            if distance < 0.5:  # Threshold to stop
                rospy.loginfo("Goal reached!")
                break

            # Compute basic control to drive the robot toward the goal
            angular_speed = atan2(goal.pose.position.y - current_pos[1], goal.pose.position.x - current_pos[0])
            twist.linear.x = 0.2
            twist.angular.z = angular_speed

            self.cmd_pub.publish(twist)

            rospy.sleep(0.1)

    def run(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            self.explore_frontiers()
            rate.sleep()

if __name__ == '__main__':
    explorer = FrontierExplorer()
    explorer.run()

# Loi khong lay duoc robot position
# 