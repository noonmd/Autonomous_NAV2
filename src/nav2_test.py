#! /usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
import math
def main():
    # --- Init
    rclpy.init()
    nav = BasicNavigator()

    # --- Set initial pose
    qx,qy,qz,qw = tf_transformations.quaternion_from_euler(0.0,0.0,math.pi)
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = nav.get_clock().now().to_msg()
    initial_pose.pose.position.x = -1.0
    initial_pose.pose.position.y = 0.0 
    initial_pose.pose.position.z = 0.0
    initial_pose.pose.orientation.x = qx
    initial_pose.pose.orientation.y = qy
    initial_pose.pose.orientation.z = qz
    initial_pose.pose.orientation.w = qw
    nav.setInitialPose(initial_pose)

    # --- Wait for Nav2
    nav.waitUntilNav2Active()

    # --- Set Goal pose
    qx,qy,qz,qw = tf_transformations.quaternion_from_euler(0.0,0.0,0.0)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = 0.0
    goal_pose.pose.position.y = -5.5
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = qx
    goal_pose.pose.orientation.y = qy
    goal_pose.pose.orientation.z = qz
    goal_pose.pose.orientation.w = qw
    nav.goToPose(goal_pose)

    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        print(feedback)
        
    # --- Set table1 pose
    qx,qy,qz,qw = tf_transformations.quaternion_from_euler(0.0,0.0,math.pi)
    table1_pose = PoseStamped()
    table1_pose.header.frame_id = 'map'
    table1_pose.header.stamp = nav.get_clock().now().to_msg()
    table1_pose.pose.position.x = -3.3
    table1_pose.pose.position.y = -1.5
    table1_pose.pose.position.z = 0.0
    table1_pose.pose.orientation.x = qx
    table1_pose.pose.orientation.y = qy
    table1_pose.pose.orientation.z = qz
    table1_pose.pose.orientation.w = qw
    nav.goToPose(table1_pose)

    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        print(feedback)
        
    # --- Set table2 pose
    qx,qy,qz,qw = tf_transformations.quaternion_from_euler(0.0,0.0,math.pi)
    table2_pose = PoseStamped()
    table2_pose.header.frame_id = 'map'
    table2_pose.header.stamp = nav.get_clock().now().to_msg()
    table2_pose.pose.position.x = -3.3
    table2_pose.pose.position.y = -2.5
    table2_pose.pose.position.z = 0.0
    table2_pose.pose.orientation.x = qx
    table2_pose.pose.orientation.y = qy
    table2_pose.pose.orientation.z = qz
    table2_pose.pose.orientation.w = qw
    nav.goToPose(table2_pose)

    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        print(feedback)
    # --- Shutdown ROS
    rclpy.shutdown()

if __name__ == '__main__':
    main()