# Cleaning Robot Project
#### @Alessio Borgi

## Introduction:
Welcome to the Cleaning Robot project designed by Alessio Borgi! This project is designed to simulate a cleaning robot using the Robot Operating System (ROS), Webots, and Rviz on Ubuntu 20.04. The cleaning robot is capable of mapping its environment and autonomously planning and navigating through it to achieve predefined cleaning goals.

## Instructions:
1. **Installations**
   - Ensure you have Ubuntu 20.04 installed on your system.
   - Install ROS (Robot Operating System) following the instructions on the [official ROS website](https://www.ros.org/install/).
   - Install Webots R2021a from the [official website](https://cyberbotics.com/#download).

2. **Workspace Setup:**
   - Create a project folder using a command like `mkdir nameFolder`.
   - Go inside the project folder typing `cd nameFolder`.
   - Create inside the project folder a Workspace and a `src` folder inside it, using the command `mkdir -p CleaningRobot_ws/src`.
   - Go inside the `src` folder of the Workspace using `cd CleaningRobot_ws/src`.
   - Clone this repository inside the `src` folder using the command `git clone `.
   - Configure ROS environment variables using the `source` command.
   - Open Webots and load the provided robot simulation environment.

3. **Launching the Project:**
   - Launch ROS nodes using the provided launch files.
   - Monitor robot behavior and visualization using Rviz.

4. **Operation:**
   - Follow the on-screen instructions to interact with the cleaning robot.
   - Use Rviz to visualize the robot's sensor data, maps, and planned paths.
   - Experiment with different cleaning goals and observe the robot's autonomous navigation.

## Functionality:
The Cleaning Robot project enables the robot to autonomously map its environment using sensor data and create a map of the house. Once the map is created, the robot can plan efficient paths to navigate through the environment and achieve predefined cleaning goals. This project demonstrates the capabilities of modern cleaning robots and showcases the integration of ROS, Webots, and Rviz for robotic applications.

Feel free to explore and modify the project to suit your needs and experiment with different robot behaviors and configurations. Happy cleaning!
