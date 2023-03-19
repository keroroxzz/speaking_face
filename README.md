# ROS Simple Face
A robot face pkg for ROS1

# What it is?

This is a simple robot face animation implementation for ROS1, which renders a robot face with OpenGL and Shader.

You can conrtol the eye and mouth movement simply through ros message. It also supports gtts and synchronize the mouth.

# Installation

    cd {your catkin_ws}/src
    git clone https://github.com/keroroxzz/speaking_face.git
    cd ..
    catkin_make

# Startup

    roslaunch speaking_face face.launch full_screen:=True

Press Esc to shutdown the GUI.