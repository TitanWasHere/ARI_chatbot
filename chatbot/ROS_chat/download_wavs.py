#!/usr/bin/env python
import rospy
import json
import os
#from pal_navigation_msgs.msg import GoToPOIActionGoal
from gtts import gTTS
import subprocess
from visualization_msgs.msg import InteractiveMarkerUpdate
from std_msgs.msg import String
# import .srv


wavs_name_dir = "andre" # prima era andre

class create_wavs:
    def __init__(self):
        rospy.init_node("create_wavs")

        # Create a service server
        # String: fileName; txt
        self.message = rospy.Service("wav_creator", String, self.callback)

        rospy.spin()

    def callback(self, req):



        req = req.data.split(";")
        name = req[0]
        text = req[1]

        tts = gTTS(text, lang='it')    
            



        mp3name = "../wavs/" + wavs_name_dir + "/" + name + ".mp3"
        wavname = "../wavs/" + wavs_name_dir + "/" + name + ".wav"
        tts.save(mp3name)

        subprocess.call(['ffmpeg', '-i', mp3name, wavname])

        os.system("aplay " + wavname)
            
        


if __name__ == "__main__":
    try:
        create_wavs()
        
    except rospy.ROSInterruptException:
        pass