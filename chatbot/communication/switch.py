#!/usr/bin/env python

import subprocess
import rospy
from std_msgs.msg import String

class Switch:
    
    def __init__(self):
        # Initialize ROS subscribers and publishers
        self.sub_message = rospy.Subscriber("/process_gpt", String, self.gpt_callback)
        self.sub_clear = rospy.Subscriber("/clear_chat", String, self.clear_chat)
        self.sub_mic = rospy.Subscriber("/microphone", String, self.listen_mic)
        
        self.pub_recognizing = rospy.Publisher("/mic_recognizing", String, queue_size=10)
        self.pub_resp = rospy.Publisher("/chatbot_response", String, queue_size=10)     
        # Start the subprocess
        self.start_process()

    def listen_mic(self, req):
        rospy.loginfo("Listening...")
        
        if self.process_mic.poll() is not None:
            rospy.logwarn("Subprocess is not running. Restarting...")
            self.start_process()

        try:
            self.process_mic.stdin.write("start" + '\n')
            self.process_mic.stdin.flush()
        except IOError as e:
            rospy.logerr("IOError while writing to subprocess stdin: %s", str(e))
            self.restart_process()

        try:
            response = self.process_mic.stdout.readline()
            if response:
                rospy.loginfo("Received response: %s", response.strip())
                self.pub_recognizing.publish(response.strip())  # Correct the publish call
            else:
                rospy.logwarn("No response received.")
        except IOError as e:
            rospy.logerr("IOError while reading from subprocess stdout: %s", str(e))
            self.restart_process()

        try:
            response = self.process_mic.stdout.readline()
            if response:
                response = response.strip()
                rospy.loginfo("Received response: %s", response)
                self.pub_resp.publish(response)  # Correct the publish call
                self.play_wav(response)
            else:
                rospy.logwarn("No response received.")
        except IOError as e:
            rospy.logerr("IOError while reading from subprocess stdout: %s", str(e))
            self.restart_process()


    def play_wav(self, msg):
        message = msg.data
        rospy.loginfo("Playing wav...")

        if self.process_wav.poll() is not None:
            rospy.logwarn("Subprocess is not running. Restarting...")
            self.start_process()

        try:
            self.process_wav.stdin.write(message + '\n')
            self.process_wav.stdin.flush()
        except IOError as e:
            rospy.logerr("IOError while writing to subprocess stdin: %s", str(e))
            self.restart_process()

        try:
            response = self.process_wav.stdout.readline()
            if response:
                rospy.loginfo("Received response: %s", response.strip())
            else:
                rospy.logwarn("No response received.")
        except IOError as e:
            rospy.logerr("IOError while reading from subprocess stdout: %s", str(e))
            self.restart_process()

    def clear_chat(self, req):
        myStr = String()
        myStr.data = "clear"
        self.gpt_callback(myStr)



    def start_process(self):
        
        rospy.loginfo("Starting subprocesses.")
        try:
            self.process = subprocess.Popen(
                ['python3.10', 'chat_process.py'],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True,
                bufsize=1
            )

            self.process_mic = subprocess.Popen(
                ['python3.10', 'mic.py'],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True,
                bufsize=1
            )

            self.process_wav = subprocess.Popen(
                ['python3.10', 'play_wav.py'],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True,
                bufsize=1
            )

        except OSError as e:
            rospy.logerr("OSError while starting subprocesses: %s", str(e))

        # Read the stderr to check for errors
        err_output = self.process.stderr.read()
        if err_output:
            rospy.logerr("Subprocess error output: %s", err_output)

        err_output_mic = self.process_mic.stderr.read()
        if err_output_mic:
            rospy.logerr("Subprocess error output: %s", err_output_mic)

        err_output_wav = self.process_wav.stderr.read()
        if err_output_wav:
            rospy.logerr("Subprocess error output: %s", err_output_wav)



    def gpt_callback(self, msg):
        message = msg.data
        rospy.loginfo("Received message: %s", message)

        # Check if the subprocess is still alive
        if self.process.poll() is not None:
            rospy.logwarn("Subprocess is not running. Restarting...")
            self.start_process()

        # Send message to subprocess
        try:
            self.process.stdin.write(message + '\n')
            self.process.stdin.flush()
        except IOError as e:
            rospy.logerr("IOError while writing to subprocess stdin: %s", str(e))
            self.restart_process()  # Restart the process if there's an error

        # Read response from subprocess
        try:
            response = self.process.stdout.readline()
            if response:
                response = response.strip()
                rospy.loginfo("Received response: %s", response)
                self.pub_resp.publish(response)
                self.play_wav(response)
            else:
                rospy.logwarn("No response received.")
        except IOError as e:
            rospy.logerr("IOError while reading from subprocess stdout: %s", str(e))
            self.restart_process()  # Restart the process if there's an error

    def restart_process(self):
        # Terminate the existing process and start a new one
        if self.process:
            try:
                self.process.terminate()
                self.process.wait()
                rospy.loginfo("Previous subprocess terminated with return code: %d", self.process.returncode)
            except Exception as e:
                rospy.logerr("Exception while terminating process: %s", str(e))
        
        self.start_process()

def main():
    # Initialize the ROS node
    try:
        rospy.init_node("switch", disable_signals=True)
        switch = Switch()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
