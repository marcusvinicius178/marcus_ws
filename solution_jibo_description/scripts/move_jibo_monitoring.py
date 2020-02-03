#! /usr/bin/env python

import rospy
import time
from math import pi, sin, cos, acos
import random
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

class jiboJointMover(object):

	
	def __init__(self):
	
		rospy.loginfo("Jibo JointMover Initializing...")
	
		self.pub_base_waist_joint_position = rospy.Publisher('/jibo2/base_waist_joint_position_controller/command', Float64, queue_size=1)
		self.pub_waist_body_joint_position = rospy.Publisher('/jibo2/waist_body_joint_position_controller/command', Float64, queue_size=1)
		self.pub_body_head_joint_position = rospy.Publisher('/jibo2/body_head_joint_position_controller/command', Float64, queue_size=1)

		self.joint_states_topic_name = "/jibo2/joint_states"
		jibo_joints_data = self._check_joint_states_ready()
		if jibo_joints_data is not None:
			self.jibo_joint_dictionary = dict(zip(jibo_joints_data.name, jibo_joints_data.position))
			rospy.Subscriber(self.joint_states_topic_name, JointState, self.jibo_joints_callback)
	

	def _check_joint_states_ready(self):
		self.joint_states = None
		rospy.logdebug("wating for "+str(self.joint_states_topic_name)+"to be Ready...")
		while self.joint_states is None and not rospy.is_shutdown():
		 	try:
				self.joint_states = rospy.wait_for_message(self.joint_states_topic_name, JointState, timeout=5.0)
				rospy.logdebug("Current "+str(self.joint_states_topic_name)+" READY=>")

			except:
				rospy.logerr("Current "+str(self.joint_states_topic_name)+" not ready yet, retrying")
			return self.joint_states


	def move_jibo_waist(self,waist_yaw_angle):

		waist_yaw = Float64()
		waist_yaw.data = waist_yaw_angle

		self.pub_base_waist_joint_position.publish(waist_yaw_angle)

	def move_jibo_body(self, body_yaw_angle):

		body_yaw = Float64()
		body_yaw.data = body_yaw_angle	

		self.pub_waist_body_joint_position.publish(body_yaw_angle)


	def move_jibo_head(self,head_yaw_angle):

		head_yaw = Float64()
		head_yaw.data = head_yaw_angle

		self.pub_body_head_joint_position.publish(head_yaw_angle)

	def jibo_joints_callback(self,msg):

		self.jibo_joint_dictionary = dict(zip(msg.name, msg.position))

	def turn_jibo_waist_left(self):

		waist_yaw_angle = -1.0

		self.move_jibo_waist(waist_yaw_angle)

	rospy.loginfo("Jibo is looking for faces at his left...")

	def turn_jibo_waist_right(self):

		waist_yaw_angle = 1.0

		self.move_jibo_waist(waist_yaw_angle)

	rospy.loginfo("Jibo is looking for faces at his right...")

	def turn_jibo_body_left(self):

		body_yaw_angle = -3.0

		self.move_jibo_body(body_yaw_angle)

	rospy.loginfo("Jibo is really making an effot to locate a face at Left...")

	def turn_jibo_body_right(self):

		body_yaw_angle = 3.0

		self.move_jibo_body(body_yaw_angle)

	rospy.loginfo("Jibo is really making an effot to locate a face at Right...")

	def turn_jibo_head_left_up(self):

		head_yaw_angle = -1.0

		self.move_jibo_head(head_yaw_angle)

	rospy.loginfo("Jibo is looking for face over his head at Left")

	def turn_jibo_head_right_up(self):

		head_yaw_angle = 1.0

		self.move_jibo_head(head_yaw_angle)

	rospy.loginfo("Jibo is looking for face over his head at Right.")

	def start_face_recognition(self):

		self.turn_jibo_waist_right()
		rospy.sleep(5.)
		self.turn_jibo_waist_left()
		rospy.sleep(5.)
		self.turn_jibo_body_right()
		rospy.sleep(5.)
		self.turn_jibo_body_left()
		rospy.sleep(5.)
		self.turn_jibo_head_left_up()
		rospy.sleep(5.)
		self.turn_jibo_head_right_up()
		rospy.sleep(5.)

	rospy.loginfo("Ok Jibo has found X faces after works")



if __name__ == "__main__":
	rospy.init_node('jointmover_jibo', anonymous=True)
	jibo_jointmover_object = jiboJointMover()
	jibo_jointmover_object.start_face_recognition()

	

	



