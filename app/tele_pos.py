#!/usr/bin/env python
"""
tele_pos.py
This module initializes a ROS node that publishes information based on the
commands sent from Server.py
"""

import rospy as rp
import socket
import time
import json
import tf
# import roslib

from math import *
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalID
from lilisocket import LiliSocket
from qrtransform import quaternion2rpy, rpy2quaternion

FIXED_COVARIANCE = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
					0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
					0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
					0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
					0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
BUFFER_SIZE = 1024

def init_socket():
	"""Initializes a LiliSocket and returns it
	"""
	to_ret = LiliSocket(socket.AF_INET, socket.SOCK_STREAM)

	while True:
		try:
			to_ret.bind(('localhost', 5555))
			to_ret.listen(1)
			return to_ret
		except socket.error as e:
			print str(e)
			t = 15
			print "Trying again in %d seconds" % t
			time.sleep(t)

def create_pose_msg(d, seq, m):
	"""Given coordinate data `d`, number of iterations `seq`, and a pose message
	`m`, set the values of `m` corresponding to the fields in `d`.
	"""
	dic = rpy2quaternion(0, 0, d['yaw'])
	m.header.seq = seq
	m.header.stamp = rp.Time.now() - rp.Duration(5)
	m.pose.position.x = d['x']
	m.pose.position.y = d['y']
	m.pose.position.z = d['z']
	m.pose.orientation.x = 0.0
	m.pose.orientation.y = 0.0
	m.pose.orientation.z = dic['z']
	m.pose.orientation.w = dic['w']

def read_recent_pose():
	"""
	"""
	pass

def main():
	rp.init_node('tele_pos')

	pub_goal = rp.Publisher('/move_base_simple/goal',
	                        PoseStamped,
							queue_size=10)
	pub_initial = rp.Publisher('/initialpose',
	                           PoseWithCovarianceStamped,
							   queue_size=10)
	pub_cancel = rp.Publisher('/move_base/cancel',
	                          GoalID,
							  queue_size=10)
	listener_pose = tf.TransformListener()
	seq_goal = 0
	seq_initial = 0

	with init_socket() as s:
		while not rp.is_shutdown():
			conn, addr = s.accept() # TODO: might need to close conn
			data = conn.recv(BUFFER_SIZE)
			while data:
				data = json.loads(data)

				if data['name'] == 'goal_map':
					message = PoseStamped()
					create_pose_msg(data, seq_goal, message)
					message.header.frame_id = 'map'
					pub_goal.publish(message)
					seq_goal += 1
				elif data['name'] == 'initial':
					message = PoseWithCovarianceStamped()
					create_pose_msg(data, seq_initial, message)
					message.header.frame_id = 'map'
					message.pose.covariance = FIXED_COVARIANCE
					pub_initial.publish(message)
					seq_initial += 1
				elif data['name'] == 'get_pose':
					pose = read_recent_pose()
					conn.sendall(pose)
				elif data['name'] == 'goal_robot':
					message = PoseStamped()
					create_pose_msg(data, seq_goal, message)
					message.header.frame_id = 'base_link'
					pub_goal.publish(message)
					seq_goal += 1
				elif data['name'] == 'cancel':
					pub_cancel.publish(GoalID())

				data = conn.recv(BUFFER_SIZE)
			print "Disconnected"



if __name__ == "__main__":
	try:
		main()
		# print len(FIXED_COVARIANCE)
	except rp.ROSInterruptException:
		pass
