# ENPM690_hw3
ENPM 690: Homewework3

Package - turtlebot_controller

# Overview

The current package has been edited to implement the teleoperation and obstacle avoidance algorith using turtlebot3. Please make sure you have turtlebot3 installed in humble space.

# Downloading and building

1. Download the package and unzip it. Make sure you have the src folder and a bunch of shell scripts along with a Dockerfile.

2. Go into the unzipped folder put the package in your workspace. Build the package
```
colcon build

```
# Launch the turtlebot3 

```
 ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py


```

# Part 1: Robot Teleoperation

1. Run the  command to teleop once the gazebo simulation is running.

```
 ros2 run turtlebot_controller teleop 

```### ▶️ Teleoperation Demo
[![Watch on YouTube]([https://img.youtube.com/vi/YOUR_VIDEO_ID/0.jpg)](https://youtu.be/tAV2gOcpOVM])

https://youtu.be/tAV2gOcpOVM

# Part 2: Autonomous Behavior


1. Relaunch the gazebo simulation and run the  command to run the autonomous mode once the gazebo simulation is running.

```
 ros2 run turtlebot_controller autonomous 

```


#Software and Packages Used:
ROS Humble
gazebo
turtlebot3

