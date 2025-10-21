import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr

class SpeechInputNode(Node):
    def __init__(self):
        super().__init__('speech_input_node')

        self.query_publisher = self.create_publisher(String, 'user_query', 10)
        self.command_publisher = self.create_publisher(String, 'speech_command', 10)

        self.recognizer = sr.Recognizer()
        
        self.recognizer.pause_threshold = 1.0
        self.recognizer.energy_threshold = 300
        self.recognizer.dynamic_energy_threshold = False
        self.recognizer.non_speaking_duration = 0.5
        
        self.microphone = sr.Microphone()
        
        self.get_logger().info("--- Speech Input Node Started ---")
        self.get_logger().info(">>> Listening... Speak clearly and pause after each phrase.")

    def listen_and_publish(self):
        """Continuously listens for speech and publishes it."""
        while rclpy.ok():
            try:
                self.get_logger().info("Ready - speak now...")
                
                with self.microphone as source:
                    # Listen with your original settings
                    audio = self.recognizer.listen(
                        source, 
                        timeout=None, 
                        phrase_time_limit=8
                    )

                text = None
                try:
                    text = self.recognizer.recognize_google(
                        audio,
                        language='en-US'
                    ).lower()
                    self.get_logger().info(f"✓ Heard (US): '{text}'")
                except sr.UnknownValueError:
                    try:
                        text = self.recognizer.recognize_google(
                            audio,
                            language='en-IN'
                        ).lower()
                        self.get_logger().info(f"✓ Heard (IN): '{text}'")
                    except sr.UnknownValueError:
                        raise

                if text:
                    if "end" in text:
                        self.publish_command("end")
                        break
                    elif "stop" in text:
                        self.publish_command("stop")
                    else:
                        self.publish_query(text)

            except sr.UnknownValueError:
                self.get_logger().warn("⚠ Could not understand audio, please try again.")
            except sr.RequestError as e:
                self.get_logger().error(f"Google Speech Recognition service error; {e}")
            except Exception as e:
                self.get_logger().error(f"An unexpected error occurred: {e}")
                break

    def publish_command(self, command_text):
        msg = String()
        msg.data = command_text
        self.command_publisher.publish(msg)
        self.get_logger().info(f"Publishing command: '{command_text}'")

    def publish_query(self, query_text):
        msg = String()
        msg.data = query_text
        self.query_publisher.publish(msg)
        self.get_logger().info(f"Publishing query: '{query_text}'")

def main(args=None):
    rclpy.init(args=args)
    node = SpeechInputNode()
    node.listen_and_publish()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()