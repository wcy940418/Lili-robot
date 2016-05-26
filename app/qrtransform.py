#!/usr/bin/env python

from math import *

def quaternion2rpy(w, x, y, z, form = 'Deg'):
	roll = atan2((-2 * y * z + 2 * x * w), (1 - 2 * x * x - 2 * y * y))
	pitch = asin(-2 * x * z + 2 * w * y)
	yaw = atan2((2 * x * y + 2 * w * z), (-2 * y * y - 2 * z * z + 1))
	if form == 'Deg':
		roll = degrees(roll)
		pitch = degrees(pitch)
		yaw = degrees(yaw)
	if yaw<0: yaw = 360 + yaw
	return {'roll': roll, 'pitch': pitch, 'yaw': yaw}

def rpy2quaternion(roll, pitch, yaw, form = 'Deg'):
	if form == 'Deg':
		roll = radians(roll) / 2
		pitch = radians(pitch) / 2
		yaw = radians(yaw) / 2
	elif form == 'Rad':
		roll = roll / 2
		pitch = pitch / 2
		yaw = yaw / 2
	w = cos(roll) * cos(pitch) * cos(yaw) + sin(roll) * sin(pitch) * sin(yaw)
	x = sin(roll) * cos(pitch) * cos(yaw) - cos(roll) * sin(pitch) * sin(yaw)
	y = cos(roll) * sin(pitch) * cos(yaw) + sin(roll) * cos(pitch) * sin(yaw)
	z = cos(roll) * cos(pitch) * sin(yaw) - sin(roll) * sin(pitch) * cos(yaw)
	return {'w': w, 'y': y, 'z': z, 'x': x}
'''
for i in range(360):
	_degree = i / 2
	sin_degree = sin(radians(_degree))
	cos_degree = cos(radians(_degree))
	dic = rpy2quaternion(0,0,i)
	D_A = quaternion2rpy(cos_degree, 0 ,0 , sin_degree)
	F_A = quaternion2rpy(dic['w'], dic['x'], dic['y'], dic['z'])
	print "Angle: %s Direct : z: %s w: %s  D_A: %s Func: z: %s w: %s F_A: %s" % (i, sin_degree, cos_degree, D_A['yaw'], dic['z'],dic['w'], F_A['yaw'])
'''