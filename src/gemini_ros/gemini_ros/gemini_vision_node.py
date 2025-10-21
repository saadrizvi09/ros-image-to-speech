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
import cv2

load_dotenv()

class GeminiVisionNode(Node):
    
    def __init__(self):
        super().__init__('gemini_vision_node')
        self.get_logger().info("Gemini Vision Node has started.")

        self.declare_parameter('use_camera', True)
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('image_path', '')
        self.declare_parameter('language', 'en')
        
        use_camera = self.get_parameter('use_camera').get_parameter_value().bool_value
        camera_index = self.get_parameter('camera_index').get_parameter_value().integer_value
        image_path = self.get_parameter('image_path').get_parameter_value().string_value
        self.language = self.get_parameter('language').get_parameter_value().string_value

        self.audio_thread = None
        self.stop_audio_flag = False
        
        # Only subscribe to user queries - no commands
        self.query_subscriber = self.create_subscription(
            String, 'user_query', self.query_callback, 10)
        self.response_publisher = self.create_publisher(String, 'ai_response', 10)
        
        try:
            genai.configure(api_key=os.environ.get("GOOGLE_API_KEY"))
            
            # Initialize camera or load image
            if use_camera:
                self.get_logger().info(f"Initializing camera {camera_index}...")
                self.camera = cv2.VideoCapture(camera_index)
                
                if not self.camera.isOpened():
                    raise ValueError(f"Could not open camera {camera_index}")
                
                # Capture initial image
                self.get_logger().info("Capturing image from camera...")
                ret, frame = self.camera.read()
                
                if not ret:
                    raise ValueError("Failed to capture image from camera")
                
                # Convert BGR to RGB
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(frame_rgb)
                
                self.get_logger().info("Image captured successfully!")
            else:
                if not image_path:
                    raise ValueError("Either use_camera must be True or image_path must be provided")
                
                self.get_logger().info(f"Loading image from: {image_path}")
                img = Image.open(image_path).convert('RGB')
                self.camera = None
            
            # Prepare image for Gemini
            img_byte_arr = io.BytesIO()
            img.save(img_byte_arr, format='PNG')
            self.image_part = {
                'mime_type': 'image/png',
                'data': img_byte_arr.getvalue()
            }
            
            self.model = genai.GenerativeModel('gemini-2.0-flash-exp')
            self.chat = self.model.start_chat(history=[])
            
            response = self.chat.send_message([self.image_part, "Provide a concise summary of what you see in this image."])
            initial_description = response.text.replace("*", "").strip()
            
            self.get_logger().info(f"AI: {initial_description}")
            self._publish_and_speak(initial_description)
            self.get_logger().info("--- Ready for queries. Use Ctrl+C to stop. ---")
            
        except Exception as e:
            self.get_logger().error(f"Initialization failed: {e}")
            if hasattr(self, 'camera') and self.camera:
                self.camera.release()
            raise

    def query_callback(self, msg):
        user_query = msg.data
        self.get_logger().info(f"You: {user_query}")
        
        try:
            # Stop current audio before responding
            self._stop_audio()
            
            # Check if user wants to recapture image
            recapture_keywords = ["new image", "take picture", "capture", "new photo", "retake"]
            if any(keyword in user_query.lower() for keyword in recapture_keywords):
                if self.camera and self.camera.isOpened():
                    self.get_logger().info("📸 Capturing new image from camera...")
                    ret, frame = self.camera.read()
                    
                    if ret:
                        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        img = Image.fromarray(frame_rgb)
                        
                        img_byte_arr = io.BytesIO()
                        img.save(img_byte_arr, format='PNG')
                        self.image_part = {
                            'mime_type': 'image/png',
                            'data': img_byte_arr.getvalue()
                        }
                        
                        # Reset chat with new image
                        self.chat = self.model.start_chat(history=[])
                        response = self.chat.send_message([self.image_part, "Provide a concise summary of what you see in this image."])
                        ai_response = response.text.replace("*", "").strip()
                    else:
                        ai_response = "Failed to capture new image from camera."
                else:
                    ai_response = "Camera is not available. Using previous image."
            else:
                # Normal query
                response = self.chat.send_message(user_query)
                ai_response = response.text.replace("*", "").strip()
            
            self.get_logger().info(f"AI: {ai_response}")
            self._publish_and_speak(ai_response)
            
        except Exception as e:
            self.get_logger().error(f"Failed: {e}")

    def _stop_audio(self):
        """Stop any currently playing audio"""
        self.stop_audio_flag = True
        sd.stop()
        if self.audio_thread and self.audio_thread.is_alive():
            self.audio_thread.join(timeout=0.5)
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
    
    def __del__(self):
        if hasattr(self, 'camera') and self.camera:
            self.camera.release()

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