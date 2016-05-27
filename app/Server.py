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
	self.service_lock = False
	self.now_service = 'No'
	self.map_mgr = MapManager()
	self.pose_svr = PoseServer()
	self.map = None

def init_request_socket():
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.bind(('192.168.3.3', MAIN_SERVER_PORT))
	s.listen(1)
	return s

def main():
	c = Config()
	s = init_request_socket()
	while True:
		with s.accept() as conn, addr:
			print 'Connected by:', addr
			data = conn.recv(1024)
			while data:
				req = json.loads(data)
				request_process(req)
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

def start_slam():
	global service_lock
	global now_service
	global slam_process
	global amcl_process
	global pose1
	global map1
	if service_lock:
		try: # reset
			os.killpg(os.getpgid(amcl_process.pid), signal.SIGTERM) # assume amcl_process is running
			service_lock = False
			now_service = 'No'
		except: # exception raised when amcl_process isn't running
			raise LILINAVIServerError("Service abnormal, please restart server")
	try:
		slam_process = subprocess.Popen(SLAM_CMD, stdout = subprocess.PIPE, shell = True, preexec_fn = os.setsid)
		print "Mapping started"
		service_lock = True
		now_service = 'SLAM'
	except:
		raise LILINAVIServerError("Service abnormal, please restart server")

def stop_slam():
	global service_lock
	global now_service
	global slam_process
	global loaded_map
	global pose1
	global map1
	if service_lock and now_service=='SLAM':
		try:
			os.killpg(os.getpgid(slam_process.pid), signal.SIGTERM)
			service_lock = False
			now_service = 'No'
			print "Mapping stopped"
		except: # exception thrown when slam_process isn't running
			raise LILINAVIServerError("Service abnormal, please restart server")

def start_amcl(name): # `name` is the name of the map (w/o filename suffix)
	global service_lock
	global now_service
	global slam_process
	global amcl_process
	global pose1
	global map1
	if service_lock and now_service=='AMCL':
		try:
			os.killpg(os.getpgid(slam_process.pid), signal.SIGTERM)
			service_lock = False
			now_service = 'No'
		except:
			raise LILINAVIServerError("Service abnormal, please restart server")
	try:
		amcl_process = subprocess.Popen(AMCL_CMD+map1.loadmap4mapserver(name), stdout = subprocess.PIPE, shell = True, preexec_fn = os.setsid)
		pose1.load(map1.getmappath(name) + '.txt')
		loaded_map = map1.getmappath(name) + '.txt'
		service_lock = True
		now_service = 'AMCL'
		print "Navigation started"
	except:
		raise LILINAVIServerError("Service abnormal, please restart server")

def stop_amcl():
	global service_lock
	global now_service
	global amcl_process
	global loaded_map
	global pose1
	global map1
	if service_lock and now_service=='AMCL':
		try:
			os.killpg(os.getpgid(amcl_process.pid), signal.SIGTERM)
			loaded_map = ''
			service_lock = False
			now_service = 'No'
			print "Navigation stopped"
		except:
			raise LILINAVIServerError("Service abnormal, please restart server")

def save_map(name):
	global service_lock
	global now_service
	global pose1
	global map1
	if service_lock and now_service == 'SLAM':
		subprocess.call(MAP_SAVER_CMD+map1.buildnewmap(name), shell = True)
		map1.buildmaplib()
		print "Map " + name +"saved"

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


def request_process(request):
	global loaded_map
	global service_lock
	global now_service
	global pose1
	global map1
	if request['cmd'] =='start_slam':
		try:
			pose1.create(map1.buildnewmap(request['data']) + '.txt')
			start_slam()
			pose1.load(map1.buildnewmap(request['data']) + '.txt')
			loaded_map = map1.buildnewmap(request['data']) + '.txt'
			#time.sleep(5)
			#record_pose('home')
		except:

			return False
	elif request['cmd'] == 'stop_slam':
		try:
			if not request['data'] == '':
				save_map(request['data'])
			stop_slam()
			loaded_map = ''
		except:
			return False
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
	return True

if __name__ == '__main__':
	main()
