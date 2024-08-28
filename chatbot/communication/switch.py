import subprocess
import rospy
from std_msgs.msg import String
import os
import signal

class Switch:
    
    def __init__(self):
        # Initialize ROS subscribers and publishers
        self.sub_message = rospy.Subscriber("/process_gpt", String, self.gpt_callback)
        
        # Start the subprocess
        self.start_process()

    def start_process(self):
        # Start the subprocess
        self.process = subprocess.Popen(
            ['python3', 'chat_process.py'],  # Use 'python' for Python 2.7
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        
    def gpt_callback(self, msg):
        message = msg.data
        rospy.loginfo("Ricevuto messaggio: %s", message)

        # Check if the subprocess is still alive
        if self.process.poll() is not None:
            rospy.logwarn("Subprocess is not running. Restarting...")
            self.start_process()

        # Send message to subprocess
        try:
            self.process.stdin.write(message + '\n')
            self.process.stdin.flush()
        except IOError as e:
            rospy.logerr("IOError while writing to subprocess stdin: %s", e)
            self.restart_process()  # Restart the process if there's an error

        # Read response from subprocess
        try:
            response = self.process.stdout.readline()
            if response:
                rospy.loginfo("Risposta: %s", response.strip())
            else:
                rospy.logwarn("Nessuna risposta ricevuta.")
        except IOError as e:
            rospy.logerr("IOError while reading from subprocess stdout: %s", e)
            self.restart_process()  # Restart the process if there's an error

    def restart_process(self):
        # Terminate the existing process and start a new one
        if self.process:
            try:
                self.process.terminate()
                self.process.wait()
            except Exception as e:
                rospy.logerr("Exception while terminating process: %s", e)
        
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
