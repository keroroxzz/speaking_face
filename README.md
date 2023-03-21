![ROS-noetic](https://img.shields.io/badge/ROS-noetic-brightgreen)
![python3.7](https://img.shields.io/badge/python-v3.7-blue)

# ROS Simple Face
A robot face pkg for ROS1 noetic.

# What is this?

This is a simple robot face animation implementation for ROS1, which renders a robot face with OpenGL and Shader.

You can conrtol the eye and mouth movement simply through ros message. It also supports gtts and synchronize the mouth.

# Requirments

The package is currently developed with:

    Python  3.7.1
    gtts    2.3.1
    numpy   1.21.5
    openCV  3.4.2
    pyaudio 0.2.11
    pydub   0.25.1

Also, make sure you have the OpenGL library.
    
    apt-get install build-essential
    apt-get install libgl1-mesa-dev libglu1-mesa-dev freeglut3-dev
    apt-get install libglew1.8 libglew-dev


# Installation

    cd {your catkin_ws}/src
    git clone https://github.com/keroroxzz/speaking_face.git
    cd ..
    catkin_make

# Startup

    roslaunch speaking_face face.launch full_screen:=True

Press Esc to shutdown the GUI.

Press Space to activate/deactivate fullscreen.

# ROS Specific

## Param

    full_screen
    sample_rate

## Publish

    # speaking.py
    /speaking_face/status
    /speaking_face/heading_config
    /speaking_face/face_config
    /speaking_face/face_param
    /speaking_face/mouse_touch

## Subscribe

    # face
    /speaking_face/heading_config
    /speaking_face/face_config
    /speaking_face/face_param

    # speaking.py
    /speaking_face/text
    /camera
    /depth