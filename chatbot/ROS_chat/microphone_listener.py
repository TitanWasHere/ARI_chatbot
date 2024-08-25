import rospy
import speech_recognition as sr
from std_msgs.msg import String

class MicrophoneListener:
    def __init__(self):
        self.pub = rospy.Publisher("/text_input", String, queue_size=10)
        self.r = sr.Recognizer()
        self.mic = sr.Microphone()
        self.sub = rospy.Subscriber("/microphpne", String, self.listen)

    def listen(self):
        with self.mic as source:
            self.r.adjust_for_ambient_noise(source)
            print("Listening...")
            audio = self.r.listen(source)
            print("Recognizing...")
            try:
                question = self.r.recognize_google(audio)
                print("You said: " + question)
                self.pub.publish(question)
            except sr.UnknownValueError:
                print("Could not understand audio")
            except sr.RequestError as e:
                print("Could not request results; {0}".format(e))
            self.listen()

if __name__ == "__main__":
    try:
        rospy.init_node("microphone_listener", disable_signals=True)
        mic = MicrophoneListener()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
