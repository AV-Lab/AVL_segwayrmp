#! /usr/bin/env python
import rospy
from std_srvs.srv import Trigger, TriggerResponse
import serial
import time

def unloader_callback(request):
    try:
        arduino = serial.Serial(port='/dev/ttyACM0', baudrate=4800, timeout=1000)
        time.sleep(2)
        arduino.write("start")
        data = arduino.readline().strip()
        if data == "complete":
            arduino.close()
        return TriggerResponse(
            success=True,
            message="Unloading complete."
        )
    except:
        return TriggerResponse(
            success=False,
            message="Oooops something went wrong..."
        )


def write_to_arduino(signal):
    arduino = serial.Serial(port='/dev/ttyACM0', baudrate=4800, timeout=1000)
    time.sleep(2)
    arduino.write(signal)
    data = arduino.readline().strip()
    print data
    if data == "complete":
        print "done"
        arduino.close()

rospy.init_node('unloader')
unloader_service = rospy.Service(
    '/unloader', Trigger, unloader_callback
)
rospy.spin()
#status = write_to_arduino("start")
