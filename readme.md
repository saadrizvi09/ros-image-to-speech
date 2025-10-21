<h1 align="center">V.O.I.D. - Visual Object Identification and Description</h1>

<p align="center">
  <img alt="ROS Version" src="https://img.shields.io/badge/ROS 2-Jazzy-blue">
  <img alt="Language" src="https://img.shields.io/badge/Language-Python-orange">
  <img alt="License" src="https://img.shields.io/badge/License-MIT-green">
</p>

---

This project is a ROS 2 application that allows a user to have a real-time, voice-controlled conversation about the contents of an image. It uses Google's Gemini AI for visual understanding and generating responses, which are then spoken back to the user via text-to-speech.

## 🚀 Key Features

* 🗣️ **Voice-Controlled Queries:** Ask questions about an image using your voice.
* 🧠 **Real-time AI Responses:** Leverages the Gemini AI model for advanced image understanding.
* 🔊 **Text-to-Speech Output:** The AI's answers are spoken back to you in natural-sounding language.
* 🛑 **Interruptible Speech:** Say "stop" at any time to immediately interrupt the AI's response.
* 🚪 **Graceful Shutdown:** Say "end" to shut down the entire application.
* 🚀 **Centralized Launch:** The entire system is managed and started with a single ROS 2 launch file.

<br>

## 🛠️ System Architecture

The application runs as a distributed system with two main nodes communicating over ROS 2 topics. This modular design allows for easy extension and maintenance.

* **`speech_node` (Speech Input):**
    * Listens to the microphone for user speech.
    * Converts speech-to-text using Google's Web Speech API.
    * Publishes user questions to the `/user_query` topic.
    * Publishes control commands ("stop", "end") to the `/speech_command` topic.

* **`vision_node` (AI & Speech Output):**
    * Subscribes to `/user_query` to receive questions.
    * Subscribes to `/speech_command` to receive control commands.
    * Sends the image and the user's question to the Gemini AI API.
    * Receives the text response from the AI.
    * Uses text-to-speech to speak the response aloud.

<br>

---

## ⚙️ Installation Guide

Follow these steps to set up and run the project on your local machine.

### 1. Prerequisites
* Ubuntu 24.04 with ROS 2 Jazzy Jalisco installed.
* Python 3.12+.
* A working microphone configured on your system.

### 2. Setup
1.  **Clone the Repository**
    ```bash
    # Navigate to your ROS 2 workspace's src directory
    cd ~/ros2_ws/src
    git clone <your-repo-url>
    ```

2.  **Install Python Dependencies**
    ```bash
    pip install google-generativeai gtts sounddevice soundfile SpeechRecognition PyAudio Pillow python-dotenv --break-system-packages
    sudo apt update && sudo apt install ffmpeg
    ```

3.  **Configure API Key**
    * Create a `.env` file in the root of your ROS 2 workspace (`~/ros2_ws`).
    * Add your Google AI API key to this file:
        ```
        GOOGLE_API_KEY="YOUR_API_KEY_HERE"
        ```

4.  **Build the Workspace**
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select gemini_ros
    ```

<br>

---

## ▶️ How to Run

The entire application can be started from a single terminal using the provided launch file.

1.  **Source the Workspace**
    Open a terminal and navigate to your workspace root:
    ```bash
    cd ~/ros2_ws
    source install/setup.bash
    ```

2.  **Run the Launch File**
    **Note:** Before running, you must edit the `src/gemini_ros/launch/gemini_app.launch.py` file to set the correct path to your image.
    ```bash
    ros2 launch gemini_ros gemini_app.launch.py
    ```
The system will start, calibrate the microphone for ambient noise, and begin listening for your voice commands. To stop the application, press `Ctrl+C` in the terminal or say "end".