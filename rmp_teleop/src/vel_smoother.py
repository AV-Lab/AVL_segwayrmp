#!/usr/bin/env python
"""By Rashid Alyassi"""
import rospy
from geometry_msgs.msg import Twist
import time
#########################
x_accel = 0.35
z_accel = 1.0
slow_const = 3 # should be slowing_const
##########################
prev_time = time.time()
last_msg = None

def callback(msg):
	#print('f1')
	global prev_time, tw, x_accel, z_accel, last_msg

	last_msg = msg

	time_diff = min(time.time() - prev_time, 0.1)

	if abs(tw.linear.x - msg.linear.x)> x_accel*time_diff:  # diff is higher accel
		if tw.linear.x < msg.linear.x:
			tw.linear.x += x_accel*time_diff *(slow_const if tw.linear.x<0 else 1)
		else:
			tw.linear.x -= x_accel*time_diff *(slow_const if tw.linear.x>0 else 1)
	else:
		tw.linear.x = msg.linear.x


	if abs(tw.angular.z - msg.angular.z)> z_accel*time_diff:  # diff is higher accel
		if tw.angular.z < msg.angular.z:
			tw.angular.z += z_accel*time_diff *(slow_const if tw.angular.z<0 else 1)
		else:
			tw.angular.z -= z_accel*time_diff *(slow_const if tw.angular.z>0 else 1)
	else:
		tw.angular.z = msg.angular.z

	prev_time = time.time()
	pub.publish(tw)
	
	
def pub_prev():
	"""pub prev msg to keep hz high"""
	global prev_time, tw, x_accel, z_accel, last_msg

	time_diff = min(time.time() - prev_time, 0.1)

	if abs(tw.linear.x - last_msg.linear.x)> x_accel*time_diff:  # diff is higher accel
		if tw.linear.x < last_msg.linear.x:
			tw.linear.x += x_accel*time_diff *(slow_const if tw.linear.x<0 else 1)
		else:
			tw.linear.x -= x_accel*time_diff*(slow_const if tw.linear.x>0 else 1)
	else:
		tw.linear.x = last_msg.linear.x


	if abs(tw.angular.z - last_msg.angular.z)> z_accel*time_diff:  # diff is higher accel
		if tw.angular.z < last_msg.angular.z:
			tw.angular.z += z_accel*time_diff *(slow_const if tw.angular.z<0 else 1)
		else:
			tw.angular.z -= z_accel*time_diff *(slow_const if tw.angular.z>0 else 1)
	else:
		tw.angular.z = last_msg.angular.z

	prev_time = time.time()


	pub.publish(tw)


print("vel_smoother node, x accel limit:", x_accel)
rospy.init_node('vel_smoother')
pub = rospy.Publisher('cmd_vel_smooth', Twist, queue_size=10)

tw = Twist()
tw.linear.y=0
tw.linear.z=0
tw.angular.x=0
tw.angular.y=0
#r = rospy.Rate(100)  # 50 hz
while not rospy.is_shutdown():
#	rospy.Subscriber("cmd_vel", Twist, callback) 
	rospy.Subscriber("cmd_vel_aggressive_smooth", Twist, callback)  
	#if last_msg is not None and time.time() - prev_time>0.02:
	#	pub_prev()
	#r.sleep()
	rospy.spin()
	