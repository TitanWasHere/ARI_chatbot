#!/usr/bin/env python

import rospy
import subprocess
from std_msgs.msg import String

class RosNode:
    def __init__(self):
        rospy.init_node('ros_node', anonymous=True)

        # Initialize subprocesses
        self.processes = {
            'chat': subprocess.Popen(
                ['python3.10', 'chat.py'],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True  
            ),
            'wavs': subprocess.Popen(
                ['python3.10', 'wavs.py'],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True  
            ),
            'mic': subprocess.Popen(
                ['python3', 'mic.py'],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True     
            )
        }

        # Initialize subscribers
        self.sub_gpt = rospy.Subscriber('/topic1', String, self.gpt_chat)
        self.sub_clear = rospy.Subscriber('/clear_chat', String, self.gpt_chat)
        self.sub_wav = rospy.Subscriber('/wav_creator', String, self.wav_callback)
        self.sub_mic = rospy.Subscriber('/microphone', String, self.mic_callback)

        self.pub_chat = rospy.Publisher("/chatbot_response", String, queue_size=10)
        self.pub_wav = rospy.Publisher("/wav_creator", String, queue_size=10)

    def gpt_chat(self, msg):
        self.send_message_to_process('chat', msg.data)

    def wav_callback(self, msg):
        self.send_message_to_process('wav', msg.data)

    def mic_callback(self, msg):
        self.send_message_to_process('mic', msg.data)

    def send_message_to_process(self, topic, message):
        process = self.processes.get(topic)
        if process:
            process.stdin.write(message + '\n')
            process.stdin.flush()
        
            stdout, stderr = process.communicate()  # Adjust timeout as needed
            if stdout:
                res = stdout.strip()
                print("Output from" +  topic +": " +  res)
                self.pub_chat.publish(res)
                self.pub_wav.publish(res)
            if stderr:
                res = stderr.strip()
                print("Errors from "+ topic + ":" + res)
                self.pub_chat.publish(res)
                self.pub_wav.publish(res)


    def __del__(self):
        for process in self.processes.values():
            process.stdin.close()
            process.stdout.close()
            process.stderr.close()
            process.terminate()

if __name__ == '__main__':
    try:
        node = RosNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
