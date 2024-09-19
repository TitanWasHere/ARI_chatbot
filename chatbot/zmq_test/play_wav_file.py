import os
from gtts import gTTS
import time
import subprocess
import zmq

class PlayWavsServer:
    def __init__(self):
        context = zmq.Context()

        # Socket to receive messages
        self.socket_recv = context.socket(zmq.PAIR)
        self.socket_recv.bind("tcp://0.0.0.0:5556")

        # Socket to send messages
        self.socket_send = context.socket(zmq.PAIR)
        self.socket_send.connect("tcp://0.0.0.0:5555")

        self.start_file_server()

    def start_file_server(self):
        while True:
            message = self.socket_recv.recv_string()
            print(f"Received message: {message}")
            response = self.play_wav(message)
            print(f"Response: {response}")
            self.socket_send.send_string(response)

    def play_wav(self, msg):
        
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


        return "success"

if __name__ == "__main__":
    PlayWavsServer()
