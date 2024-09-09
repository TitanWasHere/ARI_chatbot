import os
from gtts import gTTS
import time

class PlayWavsServer:
    def __init__(self, input_file='wav_input.txt', output_file='wav_output.txt'):
        self.input_file = input_file
        self.output_file = output_file

        print(f"[INFO]: Server PlayWav started. Waiting for requests...")

        self.start_file_server()

    def start_file_server(self):
        while True:
            # Poll the input file for new requests
            if os.path.exists(self.input_file):
                with open(self.input_file, 'r') as file:
                    message = file.read().strip()

                if message:
                    response = self.play_wav(message)
                else:
                    response = "No message received"

                # Write the response to the output file
                with open(self.output_file, 'w') as file:
                    file.write(response)

                # Remove the input file after processing
                os.remove(self.input_file)

            # Wait before polling again
            time.sleep(1)

    def play_wav(self, msg):
        try:
            tts = gTTS(msg, lang='it')
            wavname = "temp.wav"
            tts.save(wavname)
            os.system(f"aplay {wavname}")
            os.remove(wavname)
            return "success"
        except Exception as e:
            return f"Error: {str(e)}"

if __name__ == "__main__":
    PlayWavsServer()
