#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest, Empty, EmptyRequest
from zed_interfaces.msg import Object, ObjectsStamped
from zed_interfaces.srv import start_object_detection 
import sys
import os
from actionlib_msgs.msg import GoalStatusArray
import time
UNLOADER_flag = True 
NUM_PPL_DETECTED = 0
LAST_VERBOSE = -100

def human_detector_verbose_callback(data):
    global LAST_VERBOSE
    global NUM_PPL_DETECTED
    if len(data.objects) > NUM_PPL_DETECTED and time.time()-LAST_VERBOSE > 20:
        f = open('./vocaliser_service_input.txt', 'w')
        f.write("Hello human, please note that I am driving autonomously.")
	f.close()
        rospy.wait_for_service("/vocaliser")
        vocaliser_client = rospy.ServiceProxy("/vocaliser", Empty)
        try:
            result = vocaliser_client()
        except rospy.ServiceException, e:
            print "vocaliser failed"
	NUM_PPL_DETECTED = len(data.objects)
	LAST_VERBOSE = time.time()


def unloader_callback(data):
    if UNLOADER_flag:
        if data.status_list[0].status == 3:
            rospy.wait_for_service("/unloader")
            unloader_client = rospy.ServiceProxy("/unloader", Trigger)
            try:
                result = unloader_client()
                if result.success:
                    UNLOADER_flag = False
            except rospy.ServiceException, e:
                print e


def listener(args):
    if args[0]: #if unloader service enabled
        rospy.init_node('listener_goal_status', anonymous=True)
        rospy.Subscriber("/move_base/status", GoalStatusArray, unloader_callback)
    if args[1]: #if vocaliser is enabled
        rospy.init_node('human_detector_verbose', anonymous=True)
        rospy.wait_for_service("/zed2/zed_node/start_object_detection")
        obj_detect_serv = rospy.ServiceProxy("/zed2/zed_node/start_object_detection", start_object_detection)
        try:
            result = obj_detect_serv(2,80,5,True, False, False, False, False, False, False, False)
        except rospy.ServiceException, e:
            print "oops, zed is already running"
        rospy.Subscriber('/zed2/zed_node/obj_det/objects', ObjectsStamped, human_detector_verbose_callback)
    rospy.spin()


if __name__ == '__main__':
    tmparr = []
    unloader_service = False
    vocaliser = False
    tmparr.extend([unloader_service, vocaliser])
    for ind in range(len(sys.argv[1:])):
        if ind<2 and sys.argv[ind+1] == str(1):
            tmparr[ind] = True
    listener(tmparr)
