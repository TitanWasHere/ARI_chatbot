#!/usr/bin/env python

import os
import sys
import time
import speech_recognition as sr
import rospy
from std_msgs.msg import String
import zmq
import subprocess
import signal
from gtts import gTTS


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
        
        #return response
        self.pub_resp.publish(response)

        if response[0] == "!":
            resp = response.split()
            response = " ".join(resp[1:])

        self.play_wav(response)
        
    
    def play_wav(self, msg):
        #self.socket_send_wav.send_string(msg)
        print(f"Playing message: {msg}")
        tts = gTTS(msg, lang='it')
        print("Saving audio file...")
        mp3name = "temp.mp3"
        print(f"Saving mp3 file: {mp3name}")
        tts.save(mp3name)
        print("Converting mp3 to wav...")

        res = subprocess.run(['ffmpeg', '-i', mp3name, '-f', 'alsa', 'default'], check=True)

        print(f"Conversion result: {res}")


        if res != 0:        
            os.system(f"rm {mp3name}")
            #os.system(f"rm {wavname}")
            return "error"
        
        print("Playing audio file...")
        os.system(f"aplay {mp3name}")


    def clear_chat(self, req):
        #rospy.loginfo("Clearing chat...")
        #self.write_to_file(self.gpt_input_file, "clear")
        self.socket_send.send_string("clear")
        response = self.socket_recv.recv()
        #response = self.read_from_file(self.gpt_output_file)
        #rospy.loginfo("Cleared chat: %s", response)
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

            port_send_gpt = "5558"
            port_recv_gpt = "5557"

            # port_send_wav = "5556"
            # port_recv_wav = "5555"



            # res = None
            # res = subprocess.check_output(['lsof', '-t', '-i', ':'+port_send_gpt], text=True)

            # if res == None:
            #     print("[DELETE]: deleting process " + str(res))
            #     os.kill(int(res), signal.SIGTERM)
            #     res = None

            # res = subprocess.check_output(['lsof', '-t', '-i', ':'+port_recv_gpt], text=True)
            # if res == None:
            #     print("[DELETE]: deleting process " + str(res))
            #     os.kill(int(res), signal.SIGTERM)
            #     res = None

            # Starting gpt.py
            #os.system('python3.10 gpt_file.py > gpt.log 2>&1 &')
            subprocess.Popen(['python3.10', 'gpt_file.py'])
            print("[INFO]: Waiting for GPT server to start...")
            time.sleep(1)
            print("[INFO]: Started GPT server (gpt.py)")
            
            # subprocess.Popen(['python3.10', 'play_wav_file.py'])
            # print("[INFO]: Waiting for Play WAV server to start...")
            # time.sleep(1)
            # print("[INFO]: Started Play WAV server (play_wav.py)")

            

            context = zmq.Context()
            # Socket to send messages
            self.socket_send = context.socket(zmq.PAIR)
            self.socket_send.connect("tcp://0.0.0.0:"+port_send_gpt)

            # Socket to receive messages
            self.socket_recv = context.socket(zmq.PAIR)
            self.socket_recv.bind("tcp://0.0.0.0:"+port_recv_gpt)

            # # Socket to send messages
            # self.socket_send_wav = context.socket(zmq.PAIR)
            # self.socket_send_wav.connect("tcp://0.0.0.0:"+port_send_wav)

            # # Socket to receive messages
            # self.socket_recv_wav = context.socket(zmq.PAIR)
            # self.socket_recv_wav.bind("tcp://0.0.0.0:"+port_recv_wav)
                                      
                                         

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
                response = self.gpt_callback(val)
                # self.write_to_file(self.gpt_input_file, val)
                # response = self.read_from_file(self.gpt_output_file)
                print("[INFO]: GPT Response: " + response)
                if response[0] == "!":
                    resp = response.split()
                    response = " ".join(resp[1:])

                self.play_wav(response)
            elif cmd == "mic":
                response = self.listen_mic()
                print("[INFO]: Mic Response: " + response)
            elif cmd == "clear":
                #self.write_to_file(self.gpt_input_file, "clear")
                print("Clearing chat...")
            elif cmd == "exit":
                print("Exiting...")
                self.close_files()
                sys.exit(0)
            else:
                print("Invalid command")
                sys.stdout.flush()
            continue


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
