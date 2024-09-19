import subprocess
import sys
#import rospy
#from std_msgs.msg import String

class Switch:
    
    def __init__(self):
        # Initialize ROS subscribers and publishers
        # self.sub_message = rospy.Subscriber("/process_gpt", String, self.gpt_callback)
        # self.sub_clear = rospy.Subscriber("/clear_chat", String, self.clear_chat)
        # self.sub_mic = rospy.Subscriber("/microphone", String, self.listen_mic)
        
        # self.pub_recognizing = rospy.Publisher("/mic_recognizing", String, queue_size=10)
        # self.pub_resp = rospy.Publisher("/chatbot_response", String, queue_size=10)    
        # Start the subprocess
        self.process = None
        self.process_mic = None
        self.process_wav = None
        self.start_process()

        while True:
            # print("Enter command: ")
            # sys.stdout.flush()
            cmd = raw_input("Enter cmd: ")#my_list2[j]
            if cmd == "gpt":
                # print("Enter value: ")
                # sys.stdout.flush()
                val = raw_input("Enter val: ")#my_list[i]
                self.gpt_callback(val)
            elif cmd == "mic":
                self.listen_mic("")
            elif cmd == "clear":
                self.clear_chat("")

            else:
                print("Invalid command")
                sys.stdout.flush()
                

    def listen_mic(self, req):
        print("[INFO]: Listening...")
        sys.stdout.flush()
        
        if self.process_mic.poll() is not None:
            print("[WARN]: Subprocess is not running. Restarting...")
            sys.stdout.flush()

            self.start_process()

        try:
            self.process_mic.stdin.write("start" + '\n')
            self.process_mic.stdin.flush()
        except IOError as e:
            print("[ERROR]: IOError while writing to subprocess stdin: %s", str(e))
            sys.stdout.flush()
            self.restart_process()

        try:
            response = self.process_mic.stdout.readline()
            if response:
                print("[INFO]: Received response: %s", response.strip())
                sys.stdout.flush()
                #self.pub_recognizing.publish(response.strip())  # Correct the publish call
                
            else:
                print("[WARN]: No response received.")
                sys.stdout.flush()
        except IOError as e:
            print("[ERROR]: IOError while reading from subprocess stdout: %s", str(e))
            sys.stdout.flush()
            self.restart_process()

        try:
            response = self.process_mic.stdout.readline()
            if response:
                response = response.strip()
                print("[INFO]: Received response: %s", response)
                sys.stdout.flush()
                #self.pub_resp.publish(response)  # Correct the publish call
                self.play_wav(response)
            else:
                print("[WARN]: No response received.")
                sys.stdout.flush()
        except IOError as e:
            print("[ERROR]: IOError while reading from subprocess stdout: %s", str(e))
            sys.stdout.flush()
            self.restart_process()


    def play_wav(self, message):
        print("[INFO]: Playing wav...")
        sys.stdout.flush()

        if self.process_wav.poll() is not None:
            print("[WARN]: Subprocess is not running. Restarting...")
            sys.stdout.flush()
            self.start_process()

        try:
            self.process_wav.stdin.write(message + '\n')
            self.process_wav.stdin.flush()
        except IOError as e:
            print("[ERROR]: IOError while writing to subprocess stdin: %s", str(e))
            sys.stdout.flush()
            self.restart_process()

        try:
            response = self.process_wav.stdout.readline()
            if response:
                print("[INFO]: Received response: %s", response.strip())
                sys.stdout.flush()
            else:
                print("[WARN]: No response received.")
                sys.stdout.flush()
        except IOError as e:
            print("[ERROR]: IOError while reading from subprocess stdout: %s", str(e))
            sys.stdout.flush()
            self.restart_process()

    def clear_chat(self, req):
        # myStr = String()
        # myStr.data = "clear"
        #self.gpt_callback(myStr)
        print("Clearing chat...")
        sys.stdout.flush()


    def start_process(self):
        # Start the subprocess
        print("[INFO]: Starting subprocess.")
        sys.stdout.flush()

        self.process = subprocess.Popen(
            ['python3.10', 'gpt.py'],  # Use 'python' for Python 2.7
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
            bufsize=1,
        )

        self.process_mic = subprocess.Popen(
            ['python3.10', 'mic.py'],  # Use 'python' for Python 2.7
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
            bufsize=1,
        )

        self.process_wav = subprocess.Popen(
            ['python3.10', 'play_wav.py'],  # Use 'python' for Python 2.7
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
            bufsize=1,

        )

        # sys.stdout.flush()
        # if err_output:
        #     print("[ERROR]: Subprocess error output: %s", err_output)
        #     sys.stdout.flush()

        # err_output_mic = self.process_mic.stderr.read()
        # if err_output_mic:
        #     print("[ERROR]: Subprocess error output: %s", err_output_mic)
        #     sys.stdout.flush()

        # err_output_wav = self.process_wav.stderr.read()
        # if err_output_wav:
        #     print("[ERROR]: Subprocess error output: %s", err_output_wav)
        #     sys.stdout.flush()



    def gpt_callback(self, msg):
        message = msg
        print("[INFO]: Received message: %s", message)
        sys.stdout.flush()

        # Check if the subprocess is still alive
        if self.process.poll() is not None:
            print("[WARN]: Subprocess is not running. Restarting...")
            sys.stdout.flush()
            self.start_process()

        # Send message to subprocess
        try:
            self.process.stdin.write(message + '\n')
            self.process.stdin.flush()
        except IOError as e:
            print("[ERROR]: IOError while writing to subprocess stdin: %s", str(e))
            sys.stdout.flush()
            self.restart_process()  # Restart the process if there's an error

        # Read response from subprocess
        try:
            response = self.process.stdout.readline()
            if response:
                response = response.strip()
                print("[INFO]: Received response: %s", response)
                sys.stdout.flush()
                #self.pub_resp.publish(response)
                #self.play_wav(response)
            else:
                print("[WARN]: No response received.")
                sys.stdout.flush()
        except IOError as e:
            print("[ERROR]: IOError while reading from subprocess stdout: %s", str(e))
            sys.stdout.flush()
            self.restart_process()  # Restart the process if there's an error

    def restart_process(self):
        # Terminate the existing process and start a new one
        if self.process:
            try:
                self.process.terminate()
                self.process.wait()
                print("[INFO]: Previous subprocess terminated with return code: %d", self.process.returncode)
                sys.stdout.flush()
            except Exception as e:
                print("[ERROR]: Exception while terminating process: %s", str(e))
                sys.stdout.flush()
        
        self.start_process()

def main():
    # Initialize the ROS node

        switch = Switch()
        #rospy.spin()
    

if __name__ == "__main__":
    main()
