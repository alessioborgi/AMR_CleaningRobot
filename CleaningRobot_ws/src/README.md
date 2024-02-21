# Cleaning Robot Project
#### @Alessio Borgi

## Introduction:
Welcome to the Cleaning Robot project designed by Alessio Borgi! This project is designed to simulate a cleaning robot using the Robot Operating System (ROS), Webots, and Rviz on Ubuntu 20.04. The cleaning robot is capable of mapping its environment and autonomously planning and navigating through it to achieve predefined cleaning goals.

## Instructions:
1. **Installations**
   - Ensure you have Ubuntu 20.04 installed on your system.
   - Install ROS1 (Robot Operating System) following the instructions on the [official ROS website](https://www.ros.org/install/).
   - Install Webots R2021a from the [official website](https://cyberbotics.com/#download).

2. **Workspace Setup:**
   - Create a project folder using a command like `mkdir nameFolder`.
   - Go inside the project folder typing `cd nameFolder`.
   - Create inside the project folder a Workspace and a `src` folder inside it, using the command `mkdir -p CleaningRobot_ws/src`.
   - Go inside the `src` folder of the Workspace using `cd CleaningRobot_ws/src`.
   - Clone this repository inside the `src` folder using the command `git clone `.
   
3. **Launching the Project:**
   - Go inside the Workspace folder using the command `cd nameFolder/CleaningRobot_ws`.
   - Build the whole Cleaning Robot Project using `catkin build`.
   - Configure ROS environment variables using the `source devel/setup.bash` command.
   - Launch the whole project using `roslaunch robot_settings master.launch`.
   - At this point two windows should be opened: one being Webots and the other being Rviz, through which you can monitor robot behavior and perform all the operations.

## Functionality:
The Cleaning Robot project enables the robot to autonomously map its environment using sensor data and create a map of the house. Once the map is created, the robot can plan efficient paths to navigate through the environment and achieve predefined cleaning goals. This project demonstrates the capabilities of modern cleaning robots and showcases the integration of ROS, Webots, and Rviz for robotic applications.

Feel free to explore and modify the project to suit your needs and experiment with different robot behaviors and configurations. Happy cleaning!
