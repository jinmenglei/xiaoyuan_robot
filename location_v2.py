#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import os
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String
import tf
import json
from threading import Thread
import time


class GoToGoal(object):
	def __init__(self):

		rospy.init_node("navigation", anonymous = False)

		self.goal_sent = False
		rospy.on_shutdown(self.shutdown)
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		rospy.loginfo("Wait for action server!")
		self.move_base.wait_for_server(rospy.Duration(5))

		rospy.Subscriber("location", Point, self.location_callback)
		rospy.Subscriber("/move_base_cancel", String, self.move_base_cancel_callback)
		rospy.Subscriber("/get_robot_status", String, self.get_robot_status_callback)
		rospy.Subscriber("/robot_mode", String, self.robot_mode_callback)
		rospy.Subscriber('/switch_ntf', String, self.switch_ntf_callback)
		self.location_pub = rospy.Publisher('/location_ntf', String, queue_size=100)
		self.get_robot_status_pub = rospy.Publisher('/get_robot_status', String, queue_size=100)

		rospy.loginfo("navigation start!")
		self.robot_status = False
		self.robot_mode = 'none'
		self.goal = None

	def robot_mode_callback(self, data):
		self.robot_mode = data.data
		pass

	def  switch_ntf_callback(self, data):
		if self.robot_mode == 'follow':
			if data.data == 'true':
				self.robot_status = True
			else:
				self.robot_status = False
		pass

	def get_robot_status_callback(self, data):
		if data.data == 'get_robot_status':
			send_msg = String()
			send_msg.data = 'false-' + str(time.time())
			if self.robot_status:
				send_msg.data = 'true-' + str(time.time())
			self.get_robot_status_pub.publish(send_msg)



	def location_thread(self):
		time.sleep(2)
		if self.goal is not None:
			self.move_base.send_goal(self.goal)
		else:
			self.robot_status = self.goal_sent = False
			return
		count = 0
		while self.goal_sent:
			count += 1
			if count <= 600:

				success = self.move_base.wait_for_result(rospy.Duration(1))
				state = self.move_base.get_state()
				if success and state == GoalStatus.SUCCEEDED:
					rospy.loginfo("reached goal!")
					send_msg = String()
					send_msg.data = str('true')
					self.location_pub.publish(send_msg)
					break
			else:
				rospy.loginfo("failed to goal!")
				send_msg = String()
				send_msg.data = str('false')
				self.location_pub.publish(send_msg)
				self.move_base.cancel_goal()

				break

		self.robot_status = self.goal_sent = False

	def move_base_cancel_callback(self, data):
		if not self.goal_sent:
			rospy.loginfo('no goal exist!!')
			return
		self.move_base.cancel_goal()
		rospy.loginfo("GoToGoal cancel goal!")
		send_msg = String()
		send_msg.data = str('false')
		self.location_pub.publish(send_msg)
		self.robot_status = self.goal_sent = False
	
	def location_callback(self, point):
		if self.goal_sent:
			rospy.loginfo("The robot is navigationning!")
			return

		self.robot_status=  self.goal_sent = True
		pos = Pose()
		pos.position.x = point.x
		pos.position.y = point.y
		pos.position.z = 0.0
		q = tf.transformations.quaternion_from_euler(0, 0, point.z)
		pos.orientation.x = q[0]
		pos.orientation.y = q[1]
		pos.orientation.z = q[2]
		pos.orientation.w = q[3]

		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose = pos
		self.goal = goal

		location_change = Thread(target = self.location_thread)
		location_change.start()


	def shutdown(self):
		self.move_base.cancel_goal()
		rospy.loginfo("game over!")
		rospy.sleep(1)

if __name__ == "__main__":
	try:
		navigator = GoToGoal()

		rospy.sleep(1)
		rospy.spin()

	except rospy.ROSInterruptException:
		rospy.loginfo("Ctrl-C caught. Quitting")



