from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gemini_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=[
        'setuptools',
        'google-generativeai',
        'python-dotenv',
        'gtts',
        'Pillow',
        'sounddevice',
        'soundfile',
        'SpeechRecognition',
    ],
    zip_safe=True,
    maintainer='saad-rizvi',
    maintainer_email='saad.rizvi@example.com',
    description='ROS2 package for vision-based Q&A using Gemini AI with speech I/O',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vision_node = gemini_ros.gemini_vision_node:main',
            'speech_node = gemini_ros.speech_input_node:main',
        ],
    },
)