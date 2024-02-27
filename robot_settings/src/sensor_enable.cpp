#include <sensor_enable.h>

/*

File: sensor_enable.cpp
Author: @AlessioBorgi
Date: 10-02-2024

Description: This file contains the implementation for the Enable Sensors Class. This will take the name of the robot, it will perform 
             an initialization of all the sensors. Moreover, here we define the TeleOperation through Keyboard for handling mechanically 
             the movements of the robot.

*/


// Constructor of SensorEnable class.
SensorEnable::SensorEnable(ros::NodeHandle* nodehandle):nh_(*nodehandle){

    // Initialize service requests
    srv_timestep.request.value = TIME_STEP; // For all sensors.
    srv_inf.request.value = INFINITY; // For all positions of motors.
    srv_zero.request.value = 0.0; // For all velocity of motors.

    // Subscribe to topics for enabling sensors, both for /model_name and command_line one.
    subscribe_name_ = nh_.subscribe("/model_name", 1, &SensorEnable::NameCallBack,this); // In order to enable many sensors with services.
    subscribe_cmd_vel_ = nh_.subscribe("/cmd_vel", 1, &SensorEnable::CmdvelCallBack,this);
}

// Callback function for /model_name topic.
void SensorEnable::NameCallBack(const std_msgs::String& msg){

    // Get the robot name.
    SensorEnable::robot_name_ = msg.data;
    
    // Initialize the sensors.
    Initialize_sensors();
}

// Callback function for /cmd_vel topic, that handles the velocity.
void SensorEnable::CmdvelCallBack(const geometry_msgs::Twist& msg){

  // Extract linear and angular velocities.
  linear_vel = msg.linear.x;
  angular_vel = msg.angular.z;

  // Calculate wheel velocities.
  srv_act.request.value = (linear_vel - angular_vel*WHEEL_BASE)/WHEEL_RADIUS;
  vec_velocity_[0].call(srv_act);
  vec_velocity_[2].call(srv_act);
  srv_act.request.value = (linear_vel + angular_vel*WHEEL_BASE)/WHEEL_RADIUS;
  vec_velocity_[1].call(srv_act);
  vec_velocity_[3].call(srv_act);
}

// Initialize sensors.
void SensorEnable::Initialize_sensors(){

    // Declare a List of sensors.
    std::vector<std::string> sensors{"/CAM" , "/Lidar" ,"/ds_left" , "/ds_right" , "/keyboard" , "/global" , "/IMU" , "/Linear_sensor" , "/Rotation_sensor"};
    
    // Make a vector of clients for the services.
    std::vector<ros::ServiceClient> vec_client; 

    // Enable sensors using services.
    for (auto sensor = sensors.begin(); sensor != sensors.end(); ++sensor){   // (auto): Automatic Type Inference.
      
        vec_client.push_back(nh_.serviceClient<webots_ros::set_int>(SensorEnable::robot_name_+ *sensor +"/enable")); // Compose the Service Name.
        ros::service::waitForService(SensorEnable::robot_name_ + *sensor + "/enable");
        vec_client.back().call(srv_timestep); // Enable with the Timestep (in our case is set to 32 milliseconds).

    }

    // Subscribe to the Keyboard Topic.
    subscribe_keyboard_ = nh_.subscribe(SensorEnable::robot_name_+"/keyboard/key", 1, &SensorEnable::KeyboardCallBack,this);

    // List of Actuators.
    std::vector<std::string> actuators{"/wheel1" , "/wheel2" ,"/wheel3" , "/wheel4" , "/linear" , "/RM"};
    
    for (auto actuator = actuators.begin(); actuator != actuators.end(); ++actuator){

        // Setting Position of Actuators. 
        vec_client.push_back(nh_.serviceClient<webots_ros::set_float>(SensorEnable::robot_name_+ *actuator +"/set_position"));
        ros::service::waitForService(SensorEnable::robot_name_ + *actuator + "/set_position");
        vec_client.back().call(srv_inf);

        // Setting Velocity of Actuators.
        vec_velocity_.push_back(nh_.serviceClient<webots_ros::set_float>(SensorEnable::robot_name_+ *actuator +"/set_velocity"));
        ros::service::waitForService(SensorEnable::robot_name_ + *actuator + "/set_velocity");
        vec_velocity_.back().call(srv_zero);

    }
}

// Callback function for /keyboard/key topic.
void SensorEnable::KeyboardCallBack(const webots_ros::Int32Stamped& msg){

    // Handle keyboard input
    teleop(msg.data);
}

// Handle Teleoperation based on keyboard input.
void SensorEnable::teleop(int key){

    // Switch case based on keyboard input.
    switch(key) {

      // ROBOT MOVEMENTS
      // UP
      case 315 :
        srv_act.request.value = 2.0;
        vec_velocity_[0].call(srv_act); 
        vec_velocity_[1].call(srv_act);
        vec_velocity_[2].call(srv_act);
        vec_velocity_[3].call(srv_act);

        srv_act.request.value = 0;
        vec_velocity_[4].call(srv_act); // linear actuator
        vec_velocity_[5].call(srv_act); // Rotary actuator
        break;

      // DOWN
      case 317 :
        srv_act.request.value = -2.0;
        vec_velocity_[0].call(srv_act);
        vec_velocity_[1].call(srv_act);
        vec_velocity_[2].call(srv_act);
        vec_velocity_[3].call(srv_act);

        srv_act.request.value = 0;
        vec_velocity_[4].call(srv_act);
        vec_velocity_[5].call(srv_act);
        break;

      // LEFT
      case 314 :
        srv_act.request.value = -1;
        vec_velocity_[0].call(srv_act);
        vec_velocity_[2].call(srv_act);
        srv_act.request.value = 1;
        vec_velocity_[1].call(srv_act);
        vec_velocity_[3].call(srv_act);

        srv_act.request.value = 0;
        vec_velocity_[4].call(srv_act);
        vec_velocity_[5].call(srv_act);
        break;
      
      // RIGHT
      case 316 :
        srv_act.request.value = 1;
        vec_velocity_[0].call(srv_act);
        vec_velocity_[2].call(srv_act);
        srv_act.request.value = -1;
        vec_velocity_[1].call(srv_act);
        vec_velocity_[3].call(srv_act);

        srv_act.request.value = 0;
        vec_velocity_[4].call(srv_act);
        vec_velocity_[5].call(srv_act);
        break;

      






      // CAMERA MOVEMENTS
      // W
      case 87 : 
        srv_act.request.value = 0.1;
        vec_velocity_[4].call(srv_act); // Linear Actuator
        srv_act.request.value = 0;
        vec_velocity_[5].call(srv_act); // Rotary Actuator
        break;

      // S
      case 83 :
        srv_act.request.value = -0.1;
        vec_velocity_[4].call(srv_act);
        srv_act.request.value = 0;
        vec_velocity_[5].call(srv_act);
        break;

      // A
      case 65 :
        srv_act.request.value = 0.4;
        vec_velocity_[5].call(srv_act);
        srv_act.request.value = 0;
        vec_velocity_[4].call(srv_act);
        break;

      // D
      case 68 :
        srv_act.request.value = -0.4;
        vec_velocity_[5].call(srv_act);
        srv_act.request.value = 0;
        vec_velocity_[4].call(srv_act);
        break;


      // If you press any other keyboard key, the robot will immediately stop. 
      default :
        srv_act.request.value = 0;
        vec_velocity_[0].call(srv_act);
        vec_velocity_[1].call(srv_act);
        vec_velocity_[2].call(srv_act);
        vec_velocity_[3].call(srv_act);
        vec_velocity_[4].call(srv_act);
        vec_velocity_[5].call(srv_act);
   }
    std::cout<<key<<std::endl;
}
     
      
// Main function
int main(int argc, char **argv){

    // Initialize ROS node with command-line arguments.
    ros::init(argc, argv, "sensor_enable");

    // Create a NodeHandle object to communicate with ROS.
    ros::NodeHandle nh;

    // Create an instance of the SensorEnable class, passing the NodeHandle object.
    SensorEnable member(&nh);

    // Enter a loop to process ROS callbacks until the node is shut down.
    ros::spin();

    // Return 0 to indicate successful completion of the program.
    return 0;
}
