#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest, Empty, EmptyRequest
from zed_interfaces.msg import Object, ObjectsStamped
from zed_interfaces.srv import start_object_detection 
import sys
import os
from actionlib_msgs.msg import GoalStatusArray
UNLOADER_flag = True

def human_detector_verbose_callback(data):
    if len(data.objects):
        f = open('./vocaliser_service_input.txt')
        f.write("Hello human, please beware of my presence.", "w")
        print len(data.objects)
        rospy.wait_for_service("/vocaliser")
        vocaliser_client = rospy.ServiceProxy("/vocaliser", Empty)
        try:
            result = vocaliser_client()
        except rospy.ServiceException, e:
            print e


def unloader_callback(data):
    global UNLOADER_flag
    if UNLOADER_flag:
        if data.status_list:
            print data.status_list[0].status
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
    if args[1]: #if navigation vocaliser is enabled
        pass
    if args[2]: #if vocalisation of human detection is enabled
        rospy.init_node('human_detector_verbose', anonymous=True)
        rospy.wait_for_service("/zed2/zed_node/start_object_detection")
        obj_detect_serv = rospy.ServiceProxy("/zed2/zed_node/start_object_detection", start_object_detection)
        try:
            result = obj_detect_serv(2,80,5,True, False, False, False, False, False, False, False)
        except rospy.ServiceException, e:
            print e

        rospy.Subscriber('/zed2/zed_node/obj_det/objects', ObjectsStamped, human_detector_verbose_callback)
    rospy.spin()


if __name__ == '__main__':
    tmparr = []
    unloader_service = False
    navigation_vocaliser = False
    human_detection_vocaliser = False
    tmparr.extend([unloader_service, navigation_vocaliser, human_detection_vocaliser])
    for ind in range(len(sys.argv[1:])):
        if ind<3 and sys.argv[ind+1] == str(1):
            tmparr[ind] = True
    listener(tmparr)
