#! /usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
import math
import tkinter as tk
from ament_index_python.packages import get_package_share_directory
import os
import subprocess

class NavigatorApp:
    def __init__(self):
        self.navigator = BasicNavigator()
        self.tk_button = tk.Tk()
        self.create_button('Goal', 0.0, -5.5)
        self.create_button('Table 1', -3.3, -1.5)
        self.create_button('Table 2', -3.3, -2.5)
        self.create_button('Home', -1.0, 0.0)
        # self.set_initial_pose()

    def create_button(self, text, x, y):
        button = tk.Button(self.tk_button, text=text, command=lambda: self.go_to_pose(x, y))
        button.pack()

    def set_initial_pose(self):
        # Your existing initial pose setup
        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(0.0, 0.0, math.pi)
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = -1.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.position.z = 0.0
        initial_pose.pose.orientation.x = qx
        initial_pose.pose.orientation.y = qy
        initial_pose.pose.orientation.z = qz
        initial_pose.pose.orientation.w = qw
        self.navigator.setInitialPose(initial_pose)

    def go_to_pose(self, x, y):
        # Your existing navigation logic
        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(0.0, 0.0, math.pi)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = qx
        goal_pose.pose.orientation.y = qy
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw
        self.navigator.goToPose(goal_pose)

    def exiting(self):
        self.tk_button.mainloop()
        self.navigator.lifecycleShutdown()

def start_app():
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
    
    app = NavigatorApp()
    app.exiting()
    rclpy.shutdown()

if __name__ == '__main__':
    start_app()