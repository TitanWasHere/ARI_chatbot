#!/usr/bin/env python3.10

import os
import sys
from gtts import gTTS
import subprocess

wavs_name_dir = "andre"  # Change this to your directory name if needed

class CreateWavs:
    def __init__(self):
        if not os.path.exists(os.path.join("..", "wavs", wavs_name_dir)):
            os.makedirs(os.path.join("..", "wavs", wavs_name_dir))
        
    def process_message(self, message):
        # Split the input message
        try:
            name, text = message.split(";", 1)
        except ValueError:
            return "Invalid message format"

        # Generate and save TTS audio
        try:
            tts = gTTS(text, lang='it')
            mp3name = os.path.join("..", "wavs", wavs_name_dir, f"{name}.mp3")
            wavname = os.path.join("..", "wavs", wavs_name_dir, f"{name}.wav")
            tts.save(mp3name)
            tts.save(wavname)

            # Convert MP3 to WAV
            subprocess.call(['ffmpeg', '-i', mp3name, wavname])

            # Play the WAV file
            os.system(f"aplay {wavname}")

            # Clean up
            os.remove(mp3name)
            os.remove(wavname)

            return f"Processed and played: {name}"
        except Exception as e:
            return f"Error: {str(e)}"

def main():
    create_wavs = CreateWavs()
    
    while True:
        try:
            # Read from standard input
            input_message = input().strip()
            if not input_message:
                continue
            
            # Process the message
            response = create_wavs.process_message(input_message)
            
            # Write the response to standard output
            print(response)
            sys.stdout.flush()
        
        except EOFError:
            # End of input (when the parent process closes the pipe)
            break

if __name__ == "__main__":
    main()
