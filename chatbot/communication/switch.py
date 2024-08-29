import subprocess
import rospy
from std_msgs.msg import String

class Switch:
    
    def __init__(self):
        # Initialize ROS subscribers and publishers
        self.sub_message = rospy.Subscriber("/process_gpt", String, self.gpt_callback)
        self.sub_clear = rospy.Subscriber("/clear_chat", String, self.clear_chat)
        
        # Start the subprocess
        self.start_process()

    def clear_chat(self, req):
        # Log the clear command being sent
        rospy.loginfo("Sending 'clear' command to subprocess.")
        self.process.stdin.write("clear\n")
        self.process.stdin.flush()

        # Read response from subprocess
        try:
            response = self.process.stdout.readline()
            rospy.loginfo("Received response: %s", response.strip())
        except IOError as e:
            rospy.logerr("IOError while reading from subprocess stdout: %s", str(e))
            self.restart_process()  # Restart the process if there's an error

    def start_process(self):
        # Start the subprocess
        rospy.loginfo("Starting subprocess.")
        self.process = subprocess.Popen(
            ['python3.10', 'chat_process.py'],  # Use 'python' for Python 2.7
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

        # Read the stderr to check for errors
        err_output = self.process.stderr.read()
        if err_output:
            rospy.logerr("Subprocess error output: %s", err_output)

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
                rospy.loginfo("Received response: %s", response.strip())
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
