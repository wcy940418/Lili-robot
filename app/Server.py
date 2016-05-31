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

class Config:
	# self.service_lock = False
	# self.curr_process = None # one of None, 'slam', or 'amcl'
	self.map_mgr = MapManager()
	self.pose_svr = PoseServer()
	self.map_name = None

	# at least one of these should be None at all times
	self.amcl_process = None
	self.slam_process = None

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
		with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
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

def request_process(cfg, request):
	"""Given a request `request` execute the corresponding function

	Returns: {bool} True if request is successful, False otherwise
	"""
	if request['cmd'] == 'start_slam':
		if start_slam(cfg):
			map_name = request['data']
			cfg.map = cfg.map_mgr.buildnewmap(map_name)
			cfg.pose_svr.create(map_name + '.txt')
			cfg.pose_svr.load(map_name + '.txt')
		#time.sleep(5)
		#record_pose('home')
	elif request['cmd'] == 'stop_slam':
		map_name = request['data']
		if map_name:
			save_map(cfg, map_name)
		stop_slam(cfg)
	elif request['cmd'] == 'start_amcl':
		start_amcl(cfg)
	elif request['cmd'] == 'stop_amcl':
		stop_amcl(cfg)
	elif request['cmd'] == 'move':
		move(cfg, request['data'])
	elif request['cmd'] == 'set_goal_raw':
		set_goal_raw(cfg, request['data'])
	elif request['cmd'] == 'set_goal':
		set_goal(cfg, request['data'])
	elif request['cmd'] == 'stop_navi':
		stop_navi(cfg)
	elif request['cmd'] == 'get_pose':
		return read_recent_pose()
	elif request['cmd'] == 'record_pose':
		record_pose(request['data'])
	elif request['cmd'] == 'initial_pose':
		set_initial_pose(request['data'])
	elif request['cmd'] == 'initial_pose_raw':
		set_initial_pose_raw(request['data'])
	return is_successful

def init_request_socket():
	"""Create and return a socket object at a fixed IP
	"""
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.bind(('192.168.3.3', MAIN_SERVER_PORT))
	s.listen(1)
	return s

def main():
	cfg = Config()
	s = init_request_socket()
	while True:
		with s.accept() as conn, addr:
			print 'Connected by:', addr
			data = conn.recv(1024)
			while data:
				req = json.loads(data)
				request_process(cfg, req)
				data = conn.recv(1024)
			print "Client closed connection"

"""
# can't these be encapsulted in a class?
global service_lock # keep track of whether process was successfully started?
service_lock = False
global now_service # keep track of current service
now_service = 'No'
global map1
map1 = MapManager()
global pose1
pose1 = PoseServer()
global loaded_map
"""

class LILINAVIServerError(StandardError):
	pass

def set_goal_raw(cfg, data):
	"""Given goal pose data `data`, send it to the pose service

	Returns: {bool} True if successful, False otherwise
	"""
	if cfg.amcl_process:
		with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
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
		with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
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
		with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
			s.connect((HOST, POSE_SERVICE_PORT))
			s.sendall( json.dumps({'name':'cancel'}) )
		return True

	print "Cannot stop navigation; navigation not running"
	return False

def set_initial_pose(data):
	global service_lock
	global now_service
	global pose1
	global map1
	if service_lock and now_service == 'AMCL':
		sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		sock.connect((HOST,POSE_SERVICE_PORT))
		try:
			dic = pose1.findall(name) # name not defined?
			dic['name'] = 'initial'
			str = json.dumps(pose1.findall(name))
		except:
			print "Do not have this pose"
		sock.sendall(str)
		sock.close()
	else:
		print "Need to start an AMCL service first"

def set_initial_pose_raw(data):
	global service_lock
	global now_service
	global pose1
	global map1
	if service_lock and now_service == 'AMCL':
		sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		sock.connect((HOST,POSE_SERVICE_PORT))
		data['name'] = 'initial'
		str = json.dumps(data)
		sock.sendall(str)
		sock.close()
	else:
		print "Need to start an AMCL service first"

def read_recent_pose():
	global service_lock
	global pose1
	global map1
	data = {}
	if service_lock:
		try:
			sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			sock.connect((HOST,POSE_SERVICE_PORT))
			data['name'] = 'get_pose'
			str = json.dumps(data)
			sock.sendall(str)
			data = sock.recv(1024)
			if data:
				sock.close()
				try:
					d = json.loads(data)
					if d['name'] == 'robot_pose':
						d.pop('name')
						return d
				except:
					print "JSON data fomat is wrong"
			else:
				print "Cannot get robot pose"
			sock.close()
		except:
			print "Cannot connect to pose server"
	else:
		print "No service running, please start AMCL or SLAM first"

def record_pose(name):
	global service_lock
	global pose1
	global map1
	if service_lock:
		pose = read_recent_pose()
		print pose
		pose1.append(name,pose)
	else:
		print "No service running, please start AMCL or SLAM first"



if __name__ == '__main__':
	main()
