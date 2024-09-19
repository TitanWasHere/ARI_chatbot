#!/usr/bin/env python3.10

import os
import subprocess
from gtts import gTTS
import sys

class PlayWavs:

    def __init__(self):
        while True:
            start = sys.stdin.readline().strip()
            if not start:
                break

            resp = self.play_wav(start)
            print(resp)
            sys.stdout.flush()

    def play_wav(self, msg):
        try:
            tts = gTTS(msg, lang='it')
            mp3name = "temp.mp3"
            wavname = "temp.wav"
            tts.save(mp3name)
            tts.save(wavname)

            res = subprocess.call(['ffmpeg', '-i', mp3name, wavname])

            if res != 0:
                
                return "error"

            os.system(f"aplay {wavname}")

            os.remove(mp3name)
            os.remove(wavname)

            return "success"
        except Exception as e:
            return f"Error: {str(e)}"
    
if __name__ == "__main__":
    play = PlayWavs()