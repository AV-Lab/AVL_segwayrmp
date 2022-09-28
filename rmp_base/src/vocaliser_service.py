#! /usr/bin/env python
import rospy
from std_srvs.srv import Empty, EmptyResponse
from gtts import gTTS
import pyttsx3
import os
import playsound 

def vocaliser_callback(request):
    with open('./vocaliser_service_input.txt') as f:
        content = f.read().strip()
    engine = pyttsx3.init()
    engine.setProperty("rate", 178)
    voices = engine.getProperty('voices')
    engine.setProperty('voices', voices[1].id)
    engine.say(content)
    engine.runAndWait()
    #vobj = gTTS(text='content', lang=language, slow=False)
    #vobj.save("temp.mp3")
    #playsound.playsound("temp.mp3")
    #os.remove("temp.mp3")
    return EmptyResponse()

rospy.init_node('vocaliser')
vocaliser_service = rospy.Service(
    '/vocaliser', Empty, vocaliser_callback
)
rospy.spin()

