# Cleaning Robot Project
#### @Alessio Borgi




## Introduction
Welcome to the Cleaning Robot project designed by Alessio Borgi! This project is designed to simulate a cleaning robot using the Robot Operating System (ROS), Webots, and Rviz on Ubuntu 20.04. The cleaning robot is capable of mapping its environment and autonomously planning and navigating through it to achieve predefined cleaning goals.




## Instructions
1. **Installations**
   - Ensure you have Ubuntu 20.04 installed on your system.
   - Install ROS1 Noetic (Robot Operating System) following the instructions on the [official ROS website](https://www.ros.org/install/).
   - Install Webots R2021a from the [official website](https://cyberbotics.com/#download).

2. **Workspace Setup:**
   - Create a project folder using a command like `mkdir nameFolder`.
   - Go inside the project folder typing `cd nameFolder`.
   - Create inside the project folder a Workspace and a `src` folder inside it, using the command `mkdir -p CleaningRobot_ws/src`.
   - Go inside the `src` folder of the Workspace using `cd CleaningRobot_ws/src`.
   - Clone this repository inside the `src` folder using the command `git clone https://github.com/alessioborgi/CleaningRobot_RP.git`.

3. **Dependencies and Packages Installation**
   - Go inside the Workspace folder and install RosDepth using `sudo apt-get install python3-rosdep`.
   - Initialise RosDepth using `sudo rosdep init`.
   - Update rosdep in such a way to look for the required versions using `rosdep update`.
   - Go in the `robot_settings` package using `cd robot_settings`. 
   - Install all the dependencies required by the package using `rosdep install --from-paths src --ignore-src-r-y`.
   - Go back to the Workspace fodler using `cd ..`. 
   - Go in the `navigation` package using `cd navigation`. 
   - Install all the dependencies required by the package using `rosdep install --from-paths src --ignore-src-r-y`.

4. **Launching the Project:**
   - Go inside the Workspace folder using the command `cd nameFolder/CleaningRobot_ws`.
   - Build the whole Cleaning Robot Project using `catkin build`.
   - Configure ROS environment variables using the `source devel/setup.bash` command.
   - Launch the whole project using `roslaunch robot_settings master.launch`.
   - At this point two windows should be opened: one being Webots and the other being Rviz, through which you can monitor robot behavior and perform all the operations.





## Robot Structure & URDF
The robot's body is a rectangular box capable of moving in four directions: front, back, left, and right. It is equipped with a Linear Actuator and a Rotary Actuator, both connected to a camera. This combination enables the robot to adjust the camera viewpoint vertically (up and down) and horizontally (left and right). Additionally, the robot is outfitted with two Distance Sensors, two GPS units, Lidar, and an IMU (Inertial Measurement Unit). It's important to note that the wheels, distance sensors, and other components are fixed relative to the center of the body box, essentially acting as fixed joints. Conversely, the camera mechanism, which moves relative to the body, is classified as a continuous joint.

<div style="text-align: center">
  <img src="images/Robot_Image.jpg" alt="Screenshot" width="1200"/>
</div>

The Robot Structure description is physically described using the **URDF (“Unified Robot Description Format”)**, being the way to physically describe the Robot to ROS. In summary, fixed parts are connected using fixed joints, while the linear_actuator (which can move up and down) and the camera_link (which can rotate over itself), are linked using a continuous joint. Also, the Navigation is classified as a dynamic/continuous joint. This is equivalent to passing local parameters to methods. Indeed, the robot_description package converts the “CleaningRobot.xacro” file in the URDF,  and places it into the Parameter Server. The “robot_state_publisher” robot node, will then extract the URDF file from the  Parameter Server and broadcast the 3D pose of the robot link to the transform library in ROS. 
 
<div style="text-align: center">
  <img src="images/Robot_URDF_Scheme.png" alt="Screenshot" width="1200"/>
</div>

We can verify that the whole Robot Setting opening a new terminal in the Workspace folder and to ask for the `rqt_gui`, using the following command: `rosrun rqt_gui rqt_gui`. If the project is correctly working, you will have the following: 

<div style="text-align: center">
  <img src="images/rqt_gui.png" alt="Screenshot" width="1200"/>
</div>



## TeleOp (Keyboard)
The first functionality provided in this project is the **TeleOperation through the Keyboard**, allowing you to control your robot movements in the Home and also the Camera Orientation and Position using keyboard inputs. This feature enables you to manually drive or manipulate your robot's movements and actions in real-time by sending commands via the keyboard.

<div style="text-align: center">
  <img src="images/TeleOp" alt="Screenshot" width="1200"/>
</div>



## Functionalities
The Cleaning Robot project enables the robot to autonomously map its environment using sensor data and create a map of the house. Once the map is created, the robot can plan efficient paths to navigate through the environment and achieve predefined cleaning goals. This project demonstrates the capabilities of modern cleaning robots and showcases the integration of ROS, Webots, and Rviz for robotic applications.

Feel free to explore and modify the project to suit your needs and experiment with different robot behaviors and configurations. Happy cleaning!
