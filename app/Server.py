#!/usr/bin/env python

import socket
import os
import signal
import subprocess
import time
import json

from poseserver import PoseServer, PoseError
from mapmanager import MapManager
from qrtransform import quaternion2rpy, rpy2quaternion

HOST = 'localhost'
POSE_SERVICE_PORT = 5555
MAIN_SERVER_PORT = 10010
SLAM_CMD = 'roslaunch lili_navi slam.launch'
MAP_SAVER_CMD = 'rosrun map_server map_saver -f '
AMCL_CMD = 'roslaunch lili_navi amcl.launch map_file:='
BUFFER_SIZE = 1024

class Config:
	# self.service_lock = False
	# self.curr_process = None # one of None, 'slam', or 'amcl'
	map_mgr = MapManager()
	pose_svr = PoseServer()
	map_name = None

	# at least one of these should be None at all times
	amcl_process = None
	slam_process = None

class LiliSocket(socket.socket):
	def __enter__(self):
		return self

	def __exit__(self, exception_type, exception_value, traceback):
		self.close()

def start_slam(cfg):
	"""Start SLAM

	Returns: {bool} True if successful, False otherwise
	"""
	if cfg.amcl_process:
		stop_amcl(cfg)

	if cfg.slam_process:
		print "SLAM is already running"
		return False

	try:
		cfg.slam_process = subprocess.Popen(SLAM_CMD,
											stdout=subprocess.PIPE,
											shell=True,
											preexec_fn=os.setsid)
	except OSError:
		# TODO: print helpful message
		print "Unable to start SLAM"
		return False
	print "SLAM started"
	# cfg.curr_process = "slam"
	return True

def stop_slam(cfg):
	"""Stop SLAM

	Returns: {bool} True if successful, False otherwise
	"""
	if cfg.slam_process:
		try:
			os.killpg(os.getpgid(cfg.slam_process.pid), signal.SIGTERM)
		except OSError:
			# TODO: print more helpful message
			print "Unable to stop SLAM"
			return False
		cfg.slam_process = None
		print "SLAM stopped"
		return True

	print "SLAM not running; cannot stop"
	return False

def save_map(cfg, map_name):
	"""Save the map created by SLAM

	Returns: {bool} True if successful, False otherwise
	"""
	if cfg.slam_process:
		subprocess.call(MAP_SAVER_CMD + map_name, shell=True)
		cfg.map_mgr.buildmaplib()
		print "Map %s saved" % map_name
		return True
	else:
		print "SLAM isn't currently running. Please start SLAM first"
		return False

def start_amcl(cfg):
	"""Start AMCL navigation

	Returns: {bool} True if successful, False otherwise
	"""
	if cfg.slam_process:
		stop_slam(cfg)

	if cfg.amcl_process:
		print "Navigation (AMCL) already running"
		return False

	# TODO: handle exceptions?
	try:
		cfg.amcl_process = subprocess.Popen(
			AMCL_CMD + cfg.map_mgr.loadmap4mapserver(cfg.map),
			stdout=subprocess.PIPE,
			shell=True,
			preexec_fn=os.setsid)
	except OSError:
		# TODO: more helpful message
		print "Unable to start AMCL"
		return False
	cfg.pose_svr.load(cfg.map_mgr.getmappath(cfg.map) + '.txt')
	print "Navigation (AMCL) started"
	return True

def stop_amcl(cfg):
	"""Stop AMCL navigation

	Returns {bool} True if successful, False otherwise
	"""
	if cfg.amcl_process:
		try:
			os.killpg(os.getpgid(cfg.amcl_process.pid), signal.SIGTERM)
		except OSError:
			# TODO: more helpful message
			print "Unable to stop AMCL"
			return False
		cfg.amcl_process = None
		cfg.map = None
		print "Navigation (AMCL) stopped"
		return True

	print "Navigation (AMCL) not running; cannot stop"
	return False

def move(cfg, data):
	"""Given coordinate data `data`, send a move command to the pose server with
	these coordinates

	Returns: {bool} True if successful, False otherwise
	"""
	if cfg.amcl_process:
		with LiliSocket(socket.AF_INET, socket.SOCK_STREAM) as s:
			s.connect((HOST, POSE_SERVICE_PORT))
			goal = {}
			goal['x'] = data['x']
			goal['y'] = data['y']
			goal['z'] = 0.
			goal['yaw'] = data['yaw']
			goal['name'] = 'goal_robot' # ???
			s.sendall(json.dumps(goal))
		return True
	else:
		print "Please start AMCL first"
		return False

def set_goal_raw(cfg, data):
	"""Given goal pose data `data`, send it to the pose service

	Returns: {bool} True if successful, False otherwise
	"""
	if cfg.amcl_process:
		with LiliSocket(socket.AF_INET, socket.SOCK_STREAM) as s:
			s.connect((HOST, POSE_SERVICE_PORT)) # TODO: can this fail?
			data['name'] = 'goal'
			pose = json.dumps(data)
			s.sendall(pose)
		return True

	print "Cannot set raw goal; navigation (AMCL) not running"
	return False

def set_goal(cfg, name):
	"""Given a destination string `name` corresponding to a known location on
	the loaded map, send the relevant data to the pose service

	Returns: {bool} True if successful, False otherwise
	"""
	if cfg.amcl_process:
		with LiliSocket(socket.AF_INET, socket.SOCK_STREAM) as s:
			s.connect((HOST, POSE_SERVICE_PORT)) # TODO: see set_goal_raw
			try:
				coords = cfg.pose_svr.findall(name)
			except PoseError as e:
				print str(e)
				return False
			coords['name'] = 'goal_map'
			s.sendall(json.dumps(coords))
			print "Going to %s" % name
			return True

	print "Cannot set goal; navigation (AMCL) not running"
	return False

def stop_navi(cfg):
	"""Send the stop command to the pose service

	Returns: {bool} True if successful, False otherwise
	"""
	if cfg.amcl_process:
		with LiliSocket(socket.AF_INET, socket.SOCK_STREAM) as s:
			s.connect((HOST, POSE_SERVICE_PORT))
			s.sendall( json.dumps({'name':'cancel'}) )
		return True

	print "Cannot stop navigation; navigation not running"
	return False

def set_initial_pose_raw(cfg, data):
	"""Given raw coordinate data `data`, send it to the pose service

	Returns: {bool} True if successful, False otherwise
	"""
	if cfg.amcl_process:
		with LiliSocket(socket.AF_INET, socket.SOCK_STREAM) as s:
			s.connect((HOST, POSE_SERVICE_PORT))
			data['name'] = 'initial'
			s.sendall(json.dumps(data))
			return True

	print "Cannot set raw initial pose; navigation (AMCL) not running"
	return False

def set_initial_pose(cfg, name):
	"""Given a known location string `name` in the loaded map, send the
	corresponding coordinate data to the pose service

	Returns: {bool} True if successful, False otherwise
	"""
	if cfg.amcl_process:
		with LiliSocket(socket.AF_INET, socket.SOCK_STREAM) as s:
			s.connect((HOST, POSE_SERVICE_PORT))
			try:
				coords = cfg.pose_svr.findall(name)
			except PoseError as e:
				print str(e)
				return False
			s.sendall(json.dumps(coords))
			return True

	print "Cannot set initial pose; navigation (AMCL) not running"
	return False

def read_recent_pose(cfg):
	"""Get the most recent pose from the pose service

	Returns: {dict} Pose data if successful, empty dictionary otherwise
	"""
	if cfg.slam_process or cfg.amcl_process:
		with LiliSocket(socket.AF_INET, socket.SOCK_STREAM) as s:
			s.connect((HOST, POSE_SERVICE_PORT))
			s.sendall( json.dumps({'name':'get_pose'}) )
			data = s.recv(BUFFER_SIZE)

		if not data:
			print "Unable to get pose"
			return {}

		if data['name'] == 'robot_pose':
			data.pop('name')
			return data
		else:
			print "Data received is not of name \"robot_pose\""
			return {}

	print "Cannot get recent pose; neither AMCL nor SLAM is running"
	return {}

def record_pose(cfg, name):
	"""Adds a pose with name `name` to the list of known poses in the pose
	server

	Returns: {bool} True if successful, False otherwise
	"""
	if cfg.slam_process or cfg.amcl_process:
		pose = json.dumps(read_recent_pose(cfg))
		cfg.pose_svr.append(name, pose)
		print "Recorded pose:", pose
		for k, v in pose.iteritems():
			print "%3s: %.4f" % (k, v)
		return True

	print "Cannot record pose; neither SLAM nor AMCL is running"
	return False

def request_process(cfg, request):
	"""Given a request `request` execute the corresponding function
	"""
	# SLAM commands
	if request['cmd'] == 'start_slam':
		if start_slam(cfg):
			map_name = request['data']
			cfg.map = cfg.map_mgr.buildnewmap(map_name)
			cfg.pose_svr.create(map_name + '.txt')
			cfg.pose_svr.load(map_name + '.txt')
	elif request['cmd'] == 'stop_slam':
		map_name = request['data']
		if map_name:
			save_map(cfg, map_name)
		stop_slam(cfg)

	# AMCL commands
	elif request['cmd'] == 'start_amcl':
		start_amcl(cfg)
	elif request['cmd'] == 'stop_amcl':
		stop_amcl(cfg)

	# initial pose setters
	elif request['cmd'] == 'initial_pose_raw':
		set_initial_pose_raw(cfg, request['data'])
	elif request['cmd'] == 'initial_pose':
		set_initial_pose(cfg, request['data'])

	# move and stop commands
	elif request['cmd'] == 'move':
		move(cfg, request['data'])
	elif request['cmd'] == 'stop_navi':
		stop_navi(cfg)

	# goal setters
	elif request['cmd'] == 'set_goal_raw':
		set_goal_raw(cfg, request['data'])
	elif request['cmd'] == 'set_goal':
		set_goal(cfg, request['data'])

	# pose getter and recorder
	elif request['cmd'] == 'get_pose':
		read_recent_pose(cfg)
	elif request['cmd'] == 'record_pose':
		record_pose(request['data'])

def init_request_socket():
	"""Create and return a socket object at a fixed IP
	"""
	s = LiliSocket(socket.AF_INET, socket.SOCK_STREAM)
	s.bind(('192.168.3.3', MAIN_SERVER_PORT))
	s.listen(1)
	return s

def main():
	cfg = Config()
	while True:
		with init_request_socket() as s:
			conn, addr = s.accept()
			print 'Connected by:', addr
			data = conn.recv(BUFFER_SIZE)
			while data:
				req = json.loads(data)
				request_process(cfg, req)
				data = conn.recv(BUFFER_SIZE)
			print "Client closed connection"

if __name__ == '__main__':
	main()
