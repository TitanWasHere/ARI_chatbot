#!/usr/bin/env python3.10

import speech_recognition as sr
from std_msgs.msg import String
import sys

class MicrophoneListener:
    def __init__(self):
        self.r = sr.Recognizer()
        self.mic = sr.Microphone()
        
        while True:
            start = sys.stdin.readline().strip()
            if not start:
                break

            if start == "start":
                resp = self.listen()
                print(resp)
                sys.stdout.flush()

    def listen(self):
        with self.mic as source:
            self.r.adjust_for_ambient_noise(source)
            #print("Listening...")
            audio = self.r.listen(source)
            print("Recognizing...")
            sys.stdout.flush()
            try:
                question = self.r.recognize_google(audio, language="it-IT")
                #print("You said: " + question)
                #self.pub.publish(question)
                return question
            except sr.UnknownValueError:
                print("Could not understand audio")
                sys.stdout.flush()
            except sr.RequestError as e:
                print("Could not request results; {0}".format(e))
                sys.stdout.flush()
            

if __name__ == "__main__":

    mic = MicrophoneListener()



