#!/usr/bin/env python3
"""By Eyad"""
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

smoothing_factor =1

def callback2(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
	global smoothing_factor
	if data.data < 0.01:
		smoothing_factor = 1
	else:
		smoothing_factor = 0.85
	rospy.loginfo(rospy.get_caller_id() + " Roughness smoothing factor = %s", smoothing_factor)


def callback(msg):
	#print('f1')
	tw=msg
	tw.linear.x = tw.linear.x*smoothing_factor
	pub.publish(tw)
	
	

rospy.init_node('roughness_smoother')
pub = rospy.Publisher('cmd_vel_roughness', Twist, queue_size=10)

tw = Twist()
tw.linear.y=0
tw.linear.z=0
tw.angular.x=0
tw.angular.y=0
#r = rospy.Rate(100)  # 50 hz
while not rospy.is_shutdown():
#	rospy.Subscriber("cmd_vel", Twist, callback) 
	rospy.Subscriber("cmd_vel", Twist, callback)  
	rospy.Subscriber("roughness_score", Float32, callback2)  

	#if last_msg is not None and time.time() - prev_time>0.02:
	#	pub_prev()
	#r.sleep()
	rospy.spin()
	