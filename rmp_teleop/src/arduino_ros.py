#! /usr/bin/env python

import rospy
#need to change mssg bellow 
from std_msgs.msg import Int32

from pyfirmata import util, Arduino
import time


def callback(msg):
   #rospy.loginfo(rospy.get_name() + " I heard %s", data.message)
   if msg.data == "3":

      board = Arduino('/dev/ttyACM0')
      it = util.Iterator(board)
      it.start()

      #to get pin number that needs to se as 1/HIGH
      #d:13:0 means, Digital no 13 as Output
      motor_pinno = board.get_pin('d:13:o')
      led_13 = board.get_pin('d:13:o')
      while True:
         led_13.write(1)
         print('Led ON')
         time.sleep(0.2)
         led_13.write(0)
         print('Led OFF')
         time.sleep(0.2)



def listener():
   #topic = rospy.get_param('~topic', 'chatter')
   #Create a subscriber with appropriate topic, custom message and name of callback function.
   rospy.Subscriber('/counter',Int32, callback)
    # Wait for messages on topic, go to callback function when new messages arrive.
   rospy.spin()


def chatter():
    pub = rospy.Publisher('/counter', Int32, queue_size=1)
    rate = rospy.Rate(2)
    count = Int32()
    count.data = 3
    while not rospy.is_shutdown(): 
        pub.publish(count)
        rate.sleep()
   
if __name__ == '__main__':
   # Initialize the node and name it.
   rospy.init_node('arduino_flag', anonymous = True)
   #chatter()
   listener()
