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
        
        # Optimized settings for better accuracy
        self.recognizer.pause_threshold = 1.0  # Slightly longer pause
        self.recognizer.energy_threshold = 100
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
                    # Longer phrase time limit for complete sentences
                    audio = self.recognizer.listen(
                        source, 
                        timeout=None, 
                        phrase_time_limit=8
                    )

                # Use language hint for better accuracy
                text = self.recognizer.recognize_google(
                    audio,
                    language='en-IN'  # English-India for better accent recognition
                ).lower()
                
                self.get_logger().info(f"✓ Heard: '{text}'")

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