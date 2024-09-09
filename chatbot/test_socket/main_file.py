
import os
import sys
import time
import speech_recognition as sr

class Switch:
    def __init__(self):
        # Define file paths for communication
        self.gpt_input_file = 'gpt_input.txt'
        self.gpt_output_file = 'gpt_output.txt'
        self.wav_input_file = 'wav_input.txt'
        self.wav_output_file = 'wav_output.txt'
        self.r = sr.Recognizer()
        self.mic = sr.Microphone()
        
        # Start external processes
        self.start_external_processes()

        self.start_command_loop()

    def listen_mic(self):
        with self.mic as source:
            self.r.adjust_for_ambient_noise(source)
            print("Listening...")
            audio = self.r.listen(source)
            print("Recognizing...")
            try:
                question = self.r.recognize_google(audio, language="it-IT")
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

            # Starting mic.py
            os.system('python3.10 mic_file.py > mic.log 2>&1 &')
            print("[INFO]: Started Microphone server (mic.py)")

            # # Starting play_wav.py
            os.system('python3.10 play_wav_file.py > play_wav.log 2>&1 &')
            print("[INFO]: Started Play WAV server (play_wav.py)")

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

    def write_to_file(self, file_path, message):
        with open(file_path, 'w') as f:
            f.write(message)

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



        os.remove(file_path)
        return response

    def read_from_file(self, file_path):
        while not os.path.exists(file_path):
            time.sleep(0.1)

        print("[INFO]: Reading from file: " + file_path)
        with open(file_path, 'r') as f:
            response = f.read().strip()
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
    switch = Switch()

if __name__ == "__main__":
    main()
