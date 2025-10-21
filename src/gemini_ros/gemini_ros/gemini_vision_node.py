import os
import io
import google.generativeai as genai
from dotenv import load_dotenv
from gtts import gTTS
from PIL import Image
import sounddevice as sd
import soundfile as sf
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

load_dotenv()

class GeminiVisionNode(Node):
    
    def __init__(self):
        super().__init__('gemini_vision_node')
        self.get_logger().info("Gemini Vision Node has started.")

        self.declare_parameter('image_path', '')
        self.declare_parameter('language', 'en')
        
        image_path = self.get_parameter('image_path').get_parameter_value().string_value
        self.language = self.get_parameter('language').get_parameter_value().string_value

        if not image_path:
            self.get_logger().error("Image path parameter not set! Shutting down.")
            raise ValueError("Image path required")
            
        self.audio_thread = None
        self.stop_audio_flag = False
        
        self.command_subscriber = self.create_subscription(
            String, 'speech_command', self.command_callback, 10)
        self.query_subscriber = self.create_subscription(
            String, 'user_query', self.query_callback, 10)
        self.response_publisher = self.create_publisher(String, 'ai_response', 10)
        
        try:
            genai.configure(api_key=os.environ.get("GOOGLE_API_KEY"))
            
            self.get_logger().info(f"Loading image from: {image_path}")
            img = Image.open(image_path).convert('RGB')
            img_byte_arr = io.BytesIO()
            img.save(img_byte_arr, format='PNG')
            self.image_part = {
                'mime_type': 'image/png',
                'data': img_byte_arr.getvalue()
            }
            
            self.model = genai.GenerativeModel('gemini-2.0-flash-exp')
            self.chat = self.model.start_chat(history=[])
            
            response = self.chat.send_message([self.image_part, "Provide a concise summary of the text content in this image."])
            initial_description = response.text.replace("*", "").strip()
            
            self.get_logger().info(f"AI: {initial_description}")
            self._publish_and_speak(initial_description)
            self.get_logger().info("--- Ready for queries ---")
            
        except Exception as e:
            self.get_logger().error(f"Initialization failed: {e}")
            raise

    def query_callback(self, msg):
        user_query = msg.data
        self.get_logger().info(f"You: {user_query}")
        
        try:
            self._stop_audio()
            response = self.chat.send_message(user_query)
            ai_response = response.text.replace("*", "").strip()
            self.get_logger().info(f"AI: {ai_response}")
            self._publish_and_speak(ai_response)
        except Exception as e:
            self.get_logger().error(f"Failed: {e}")

    def command_callback(self, msg):
        command = msg.data.lower()
        if command == 'stop':
            self._stop_audio()
        elif command == 'end':
            self._stop_audio()
            rclpy.shutdown()

    def _stop_audio(self):
        self.stop_audio_flag = True
        sd.stop()
        if self.audio_thread and self.audio_thread.is_alive():
            self.audio_thread.join(timeout=1.0)
        self.stop_audio_flag = False

    def _publish_and_speak(self, text):
        msg = String()
        msg.data = text
        self.response_publisher.publish(msg)
        self.audio_thread = threading.Thread(target=self._speak, args=(text,))
        self.audio_thread.start()
        
    def _speak(self, text):
        temp_audio = None
        try:
            tts = gTTS(text=text, lang=self.language, slow=False)
            temp_audio = f"/tmp/ros_speech_{os.getpid()}.mp3"
            tts.save(temp_audio)
            audio_data, sample_rate = sf.read(temp_audio)
            sd.play(audio_data, sample_rate)
            sd.wait()
        except Exception as e:
            self.get_logger().error(f"TTS failed: {e}")
        finally:
            if temp_audio and os.path.exists(temp_audio):
                try:
                    os.remove(temp_audio)
                except:
                    pass

def main(args=None):
    rclpy.init(args=args)
    try:
        node = GeminiVisionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()