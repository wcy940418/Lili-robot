#!/usr/bin/env python
import rospy 
import geometry_msgs.msg
import actionlib_msgs.msg
from math import *
import socket
import time
import json
import tf
import roslib
from qrtransform import *

FIXED_COVARIANCE = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]

def _socket_init():
	global s
	print 'ahaha'
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	try:
		s.bind(('localhost', 5555))
		s.listen(1)
	except:
		print "port has been occupied"
	print "pose server start sucessfully"

def _message_init():
	global pub_goal
	global pub_estimate, pub_cancel, msg_cancel
	global msg_goal, msg_initial
	pub_goal = rospy.Publisher('/move_base_simple/goal', geometry_msgs.msg.PoseStamped, queue_size = 10)
	pub_estimate = rospy.Publisher('/initialpose' ,geometry_msgs.msg.PoseWithCovarianceStamped, queue_size = 10)
	pub_cancel = rospy.Publisher('/move_base/cancel' ,actionlib_msgs.msg.GoalID, queue_size = 10)
	rospy.init_node('tele_pos', anonymous = False)
	msg_goal = geometry_msgs.msg.PoseStamped()
	msg_initial = geometry_msgs.msg.PoseWithCovarianceStamped()
	msg_cancel = actionlib_msgs.msg.GoalID()

def start_tf_listener():
	global pose_listener
	pose_listener = tf.TransformListener()

def read_recent_pose():
	global pose_listener
	dic = {}
	try:
		(trans,rot) = pose_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
		dic['name'] = 'robot_pose'
		dic['x'] = trans[0]
		dic['y'] = trans[1]
		dic['z'] = trans[2]
		dic['yaw'] = quaternion2rpy(rot[3], rot[0], rot[1], rot[2])['yaw']
		return json.dumps(dic)
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		print "Cannot get robot pose"

def tele_pos():
	_socket_init()
	while not rospy.is_shutdown():
		global pub_goal, pub_estimate, pub_cancel
		global seq_goal, seq_initial
		global msg_goal, msg_initial, msg_cancel
		global s
		global conn
		conn, addr = s.accept()
		print 'Connected by:', addr
		while True:
			data = conn.recv(1024)
			if data:
				d = json.loads(data)
				if d['name'] == 'goal_map':
					dic = rpy2quaternion(0,0,d['yaw'])
					msg_goal.header.seq = seq_goal
					msg_goal.header.stamp = rospy.Time.now() - rospy.Duration(5)
					msg_goal.header.frame_id = 'map'
					msg_goal.pose.position.x = d['x']
					msg_goal.pose.position.y = d['y']
					msg_goal.pose.position.z = d['z']
					msg_goal.pose.orientation.x = 0.0
					msg_goal.pose.orientation.y = 0.0
					msg_goal.pose.orientation.z = dic['z']
					msg_goal.pose.orientation.w = dic['w']
					seq_goal += 1
					pub_goal.publish(msg_goal)
				elif d['name'] == 'cancel':
					pub_cancel.publish(msg_cancel)
				elif d['name'] == 'initial':
					dic = rpy2quaternion(0,0,d['yaw'])
					msg_initial.header.seq = seq_initial
					msg_initial.header.stamp = rospy.Time.now() - rospy.Duration(5)
					msg_initial.header.frame_id = 'map'
					msg_initial.pose.position.x = d['x']
					msg_initial.pose.position.y = d['y']
					msg_initial.pose.position.z = d['z']
					msg_initial.pose.orientation.x = 0.0
					msg_initial.pose.orientation.y = 0.0
					msg_initial.pose.orientation.z = dic['z']
					msg_initial.pose.orientation.w = dic['w']
					msg_initial.pose.covariance = FIXED_COVARIANCE
					seq_initial += 1
					pub_initial.publish(msg_initial)
				elif d['name'] == 'get_pose':
					str = read_recent_pose()
					conn.sendall(str)
				elif d['name'] == 'goal_robot':
					dic = rpy2quaternion(0,0,d['yaw'])
					msg_goal.header.seq = seq_goal
					msg_goal.header.stamp = rospy.Time.now() - rospy.Duration(5)
					msg_goal.header.frame_id = 'base_link'
					msg_goal.pose.position.x = d['x']
					msg_goal.pose.position.y = d['y']
					msg_goal.pose.position.z = d['z']
					msg_goal.pose.orientation.x = 0.0
					msg_goal.pose.orientation.y = 0.0
					msg_goal.pose.orientation.z = dic['z']
					msg_goal.pose.orientation.w = dic['w']
					seq_goal += 1
					pub_goal.publish(msg_goal)
			else: 		#if client closes connection, conn.recv() will return a null value
				print "Client has been killed"
				break
		conn.close()
		print 'Disconnected'

def shutdown_hook():
	global s
	#global conn
	#conn.close()
	s.close()
	print "tele_pos server has shutdown safely"

if __name__ == '__main__':
	try:
		seq_goal = 0
		seq_initial = 0
		rospy.on_shutdown(shutdown_hook)
		_message_init()
		start_tf_listener()
		tele_pos()
	except rospy.ROSInterruptException:
		pass