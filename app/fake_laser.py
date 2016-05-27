#!/usr/bin/env python
import rospy 
import sensor_msgs.msg
from math import *

PI = 3.14159265358

def message_init():
	global msg_fake_laser, pub_fake_laser
	pub_fake_laser = rospy.Publisher('/fake_scan', sensor_msgs.msg.LaserScan, queue_size=10)
	rospy.init_node('clear_scan', anonymous = False)
	msg_fake_laser = sensor_msgs.msg.LaserScan()
	msg_fake_laser.ranges = [0]
	for i in range(359):
		msg_fake_laser.ranges.append(0)

def send_fake_scan(distance):
	global msg_fake_laser, pub_fake_laser, seq
	msg_fake_laser.header.seq = seq
	msg_fake_laser.header.stamp = rospy.Time.now()
	msg_fake_laser.header.frame_id = 'base_link'
	msg_fake_laser.angle_min = 0.0
	msg_fake_laser.angle_max = PI * 2
	msg_fake_laser.angle_increment = PI / 180.0
	msg_fake_laser.time_increment = 0.00001
	msg_fake_laser.scan_time = 0.0
	msg_fake_laser.range_min = 0.0
	msg_fake_laser.range_max = distance
	for i in range(360):
		msg_fake_laser.ranges[i] = distance - 0.01
	pub_fake_laser.publish(msg_fake_laser)

global msg_fake_laser, pub_fake_laser, seq


if __name__ == "__main__":
	try:
		seq = 0
		message_init()
		rate = rospy.Rate(20)
		while not rospy.is_shutdown():
			distance = rospy.get_param('~distance')
			send_fake_scan(distance)
			rate.sleep()
			seq = seq + 1
	except rospy.ROSInterruptException:
		pass



