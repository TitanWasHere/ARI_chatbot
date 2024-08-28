#!/usr/bin/env python3.10

import speech_recognition as sr
import sys

class MicrophoneListener:
    def __init__(self):
        self.r = sr.Recognizer()
        self.mic = sr.Microphone()

    def listen(self):
        with self.mic as source:
            self.r.adjust_for_ambient_noise(source)
            print("Listening...")
            audio = self.r.listen(source)
            print("Recognizing...")
            try:
                question = self.r.recognize_google(audio)
                print("You said: " + question)
                # Send recognized text to stdout
                print(question)
                sys.stdout.flush()
            except sr.UnknownValueError:
                print("Could not understand audio")
                sys.stdout.flush()
            except sr.RequestError as e:
                print("Could not request results; {0}".format(e))
                sys.stdout.flush()

def main():
    listener = MicrophoneListener()
    while True:
        try:
            # Continuously listen for audio input
            listener.listen()
        except KeyboardInterrupt:
            # Handle script termination with Ctrl+C
            print("Terminating...")
            break

if __name__ == "__main__":
    main()
