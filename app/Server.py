#!/usr/bin/env python

import socket
import os
import signal
import subprocess
import time
import json

from poseserver import PoseError, PoseServer
from mapmanager import MapError, MapManager
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
	self.map = None # name of the map

	# at least one of these should be None at all times
	self.amcl_process = None
	self.slam_process = None

def start_slam(cfg):
	"""Start SLAM

	Returns: {bool} True if successful, False otherwise
	"""
	if cfg.amcl_process:
		stop_amcl(cfg)

	# will this ever fail?
	cfg.slam_process = subprocess.Popen(SLAM_CMD,
										stdout=subprocess.PIPE,
										shell=True,
										preexec_fn=os.setsid)
	print "SLAM started"
	# cfg.curr_process = "slam"
	return True

def stop_slam(cfg):
	"""Stop SLAM

	Returns: {bool} True if successful, False otherwise
	"""
	if cfg.slam_process:
		os.killpg(os.getpgid(cfg.slam_process.pid), signal.SIGTERM)
		cfg.slam_process = None
		print "SLAM stopped"
		return True

	print "SLAM not running; cannot stop"
	return False

def save_map(cfg):
	"""Save the map created by SLAM

	Returns: {bool} True if successful, False otherwise
	"""
	if cfg.slam_process:
		subprocess.call(MAP_SAVER_CMD + cfg.map, shell=True)
		cfg.map_mgr.buildmaplib()
		print "Map %s saved" % cfg.map
		return True
	else:
		print "SLAM isn't currently running. Please start SLAM first"
		return False

def start_amcl(cfg):
	"""Start AMCL navigation

	Returns:
	"""
	if cfg.slam_process:
		stop_slam(cfg)

	# TODO: handle exceptions?
	cfg.amcl_process = subprocess.Popen(
		AMCL_CMD + cfg.map_mgr.loadmap4mapserver(cfg.map),
		stdout=subprocess.PIPE,
		shell=True,
		preexec_fn=os.setsid)

	cfg.pose_svr.load(cfg.map_mgr.getmappath(cfg.map) + '.txt')
	print "Navigation started"

def stop_amcl(cfg):
	if cfg.amcl_process:
		os.killpg(os.getpgid(cfg.amcl_process.pid), signal.SIGTERM)
		cfg.amcl_process = None
		cfg.map = None
		print "Navigation stopped"
		return True

	print "Navigation not running; cannot stop"
	return False

def request_process(cfg, req):
	"""Given a request `req` execute the corresponding function

	Returns: {bool} True if request is successful, False otherwise
	"""
	is_successful = False
	if request['cmd'] == 'start_slam':
		is_successful = start_slam(cfg)
		if not is_successful:
			return False
		cfg.map = cfg.map_mgr.buildnewmap(request['data'])
		cfg.pose_svr.create(cfg.map + '.txt')
		cfg.pose_svr.load(cfg.map + '.txt')
		#time.sleep(5)
		#record_pose('home')
	elif request['cmd'] == 'stop_slam':
		if request['data']:
			is_successful = save_map(cfg) and stop_slam(cfg)
	elif request['cmd'] == 'start_amcl':
		try:
			start_amcl(request['data'])
		except:
			return False
	elif request['cmd'] == 'stop_amcl':
		try:
			stop_amcl()
		except:
			return False
	elif request['cmd'] == 'move':
		move(request['data'])
	elif request['cmd'] == 'set_goal_raw':
		set_goal_raw(request['data'])
	elif request['cmd'] == 'set_goal':
		set_goal(request['data'])
	elif request['cmd'] == 'stop_navi':
		stop_navi()
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

def set_goal_raw(data):
	global service_lock
	global now_service
	global pose1
	global map1
	if service_lock :
		sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		sock.connect((HOST,POSE_SERVICE_PORT))
		data['name'] = 'goal'
		str = json.dumps(data)
		sock.sendall(str)
		sock.close()
	else: # doesn't start_slam set service_lock = True?
		print "Need to start an AMCL service first"

def set_goal(name):
	global service_lock
	global now_service
	global pose1
	global map1
	if service_lock :
		sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # protocol, socket type
		sock.connect((HOST,POSE_SERVICE_PORT))
		try:
			dic = pose1.findall(name)
			dic['name'] = 'goal_map'
			str = json.dumps(dic)
			sock.sendall(str)
			print "LILI will go to " + name
		except: # first line in `try`
			print "Do not have this pose"

		sock.close()
	else: # see comment for set_goal_raw
		print "Need to start an AMCL service first"

def move(data):
	global service_lock
	global now_service
	global map1
	dic = {}
	if service_lock :
		sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		sock.connect((HOST,POSE_SERVICE_PORT))
		try:
			dic['x'] = data['x']
			dic['y'] = data['y']
			dic['z'] = 0.0
			dic['yaw'] = data['yaw']
			dic['name'] = 'goal_robot'
			str = json.dumps(dic) # str is a keyword in python
		except: # what error will be thrown?
			print "Invalid position"
		sock.sendall(str)
		sock.close()
	else:
		print "Need to start an AMCL service first"

def stop_navi():
	global service_lock
	global now_service
	global pose1
	global map1
	dic = {}
	if service_lock and now_service == 'AMCL':
		sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		sock.connect((HOST,POSE_SERVICE_PORT))
		dic['name'] = 'cancel'
		str = json.dumps(dic)
		sock.sendall(str)
		sock.close()
	else:
		print "Need to start an AMCL service first"

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
