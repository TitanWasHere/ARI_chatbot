#!/usr/bin/env python

import os
import sys
import time
import speech_recognition as sr
import rospy
from std_msgs.msg import String
import zmq

class Switch:
    def __init__(self):
        # Initialize ROS subscribers and publishers
        self.sub_message = rospy.Subscriber("/process_gpt", String, self.gpt_callback)
        self.sub_clear = rospy.Subscriber("/clear_chat", String, self.clear_chat)
        self.sub_mic = rospy.Subscriber("/microphone", String, self.listen_mic)
        
        self.pub_recognizing = rospy.Publisher("/mic_recognizing", String, queue_size=10)
        self.pub_mic = rospy.Publisher("/mic_response", String, queue_size=10)
        self.pub_resp = rospy.Publisher("/chatbot_response", String, queue_size=10)    
        # Define file paths for communication
        self.gpt_input_file = 'gpt_input.txt'
        self.gpt_output_file = 'gpt_output.txt'
        self.wav_input_file = 'wav_input.txt'
        self.wav_output_file = 'wav_output.txt'
        self.r = sr.Recognizer()
        self.mic = sr.Microphone()
        
        
        
        # Clean up any existing files

        # if os.path.isfile(self.gpt_input_file):
        #     os.remove(self.gpt_input_file)

        # if os.path.isfile(self.wav_input_file):
        #     os.remove(self.wav_input_file)

        
        # Start external processes
        self.start_external_processes()

        #self.start_command_loop()

    def gpt_callback(self, req):
        rospy.loginfo("Received request: %s", req.data)
        #self.write_to_file(self.gpt_input_file, req.data)
        msg = req.data
        self.socket_send.send_string(msg)
        response = self.socket_recv.recv()
        #response = self.read_from_file(self.gpt_output_file)
        rospy.loginfo("GPT Response: %s", response)
        self.pub_resp.publish(response)

    def clear_chat(self, req):
        rospy.loginfo("Clearing chat...")
        #self.write_to_file(self.gpt_input_file, "clear")
        self.socket_send.send_string("clear")
        response = self.socket_recv.recv()
        #response = self.read_from_file(self.gpt_output_file)
        rospy.loginfo("Cleared chat: %s", response)
        self.pub_resp.publish(response)

    def listen_mic(self, req=None):
        with self.mic as source:
            self.r.adjust_for_ambient_noise(source)
            #print("Listening...")
            self.pub_recognizing.publish("Listening")
            audio = self.r.listen(source)
            #print("Recognizing...")
            self.pub_recognizing.publish("Recognizing")
            try:
                question = self.r.recognize_google(audio, language="it-IT")
                self.pub_recognizing.publish("Recognized")
                self.pub_mic.publish(question)
                return question
            except sr.UnknownValueError:
                return "Could not understand audio"
            except sr.RequestError as e:
                return "Could not request results; " + str(e)

    def start_external_processes(self):
        """Start the external Python scripts."""
        try:
            # Starting gpt.py
            os.system('python3.10 gpt_file.py > gpt.log 2>&1 &')
            print("[INFO]: Started GPT server (gpt.py)")

            context = zmq.Context()

            # Socket to send messages
            self.socket_send = context.socket(zmq.PAIR)
            self.socket_send.connect("tcp://0.0.0.0:5558")

            # Socket to receive messages
            self.socket_recv = context.socket(zmq.PAIR)
            self.socket_recv.bind("tcp://0.0.0.0:5557")

            # # Starting play_wav.py
            #os.system('python3.10 play_wav_file.py > play_wav.log 2>&1 &')
            #print("[INFO]: Started Play WAV server (play_wav.py)")

        except Exception as e:
            print("[ERROR]: Could not start external processes: " + str(e))
            sys.stdout.flush()

    def start_command_loop(self):
        while True:
            cmd = raw_input("Enter cmd: ")  # Use raw_input for Python 2.7
            if cmd == "gpt":
                val = raw_input("Enter val: ")
                self.write_to_file(self.gpt_input_file, val)
                response = self.read_from_file(self.gpt_output_file)
                print("[INFO]: GPT Response: " + response)
            elif cmd == "mic":
                response = self.listen_mic()
                print("[INFO]: Mic Response: " + response)
            elif cmd == "clear":
                self.write_to_file(self.gpt_input_file, "clear")
                print("Clearing chat...")
            elif cmd == "exit":
                print("Exiting...")
                self.close_files()
                sys.exit(0)
            else:
                print("Invalid command")
                sys.stdout.flush()
            continue

    def write_to_file(self, file_path, message):
        with open(file_path, 'w') as f:
            f.write(message)
            f.flush()

    def read_from_mic_file(self, file_path):
        while not os.path.exists(file_path):
            time.sleep(0.1)

        print("[INFO]: Reading from file: " + file_path)
        with open(file_path, 'r') as f:
            response = f.read().strip()
            print(response)
            if response == "Invalid command":
                self.close_files()
                sys.exit(1)
            response = f.read().strip()
            print(response)
            response = f.read().strip()
            print(response)
            f.flush()



        os.remove(file_path)
        return response

    def read_from_file(self, file_path):
        while not os.path.exists(file_path):
            time.sleep(0.1)

        print("[INFO]: Reading from file: " + file_path)
        with open(file_path, 'r') as f:
            response = f.read().strip()
            f.flush()
        os.remove(file_path)  # Clean up the file after reading
        return response

    def close_files(self):
        """Close and clean up files if necessary."""
        for file_path in [self.gpt_input_file, self.gpt_output_file,
                          self.mic_input_file, self.mic_output_file,
                          self.wav_input_file, self.wav_output_file, "prova.txt"]:
            if os.path.exists(file_path):
                os.remove(file_path)
        print("[INFO]: Closed and cleaned up files")

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
