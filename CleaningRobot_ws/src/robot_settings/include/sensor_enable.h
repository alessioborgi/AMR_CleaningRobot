#ifndef SENSOR_ENABLE_H
#define SENSOR_ENABLE_H


/*

File: sensor_enable.h
Author: @AlessioBorgi
Date: 10-02-2024

Description: This file contains the definition header for the Enabling Sensor class. Here, we Subscribe to the "/model_name" topic that 
             returns back the name of the Robot, that will change at every master.launch call.
             We will use this model_name in the robot_name/sensor/enable, to enable all the sensors. 
             
*/


// Include necessary libraries
#include "ros/ros.h"
#include "webots_ros/Int32Stamped.h"
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/robot_get_device_list.h>
#include <std_msgs/String.h>
#include <signal.h>
#include <stdio.h>
#include <iostream>
#include <geometry_msgs/Twist.h>

// Define constants
#define TIME_STEP 32
#define WHEEL_BASE 0.1
#define WHEEL_RADIUS 0.06


// Define the class SensorEnable
class SensorEnable{

    private:
        ros::NodeHandle nh_;                                // NodeHandle for ROS communication.
        ros::Subscriber subscribe_cmd_vel_;                 // ROS subscriber for receiving cmd_vel messages.
        ros::Subscriber subscribe_name_;                    // ROS subscriber for receiving the Robot's Name.
        ros::Subscriber subscribe_keyboard_;                // ROS subscriber for Keyboard Input.
        webots_ros::set_int srv_timestep;                   // Service message for setting time step.
        webots_ros::set_float srv_inf;                      // Service message for setting float value.
        webots_ros::set_float srv_zero;                     // Service message for setting float value to zero.
        webots_ros::set_float srv_act;                      // Service message for setting float value.
        std::vector<ros::ServiceClient> vec_velocity_;      // Vector to store ROS service clients.
        std::string robot_name_;                            // Variable to store the robot's name.
        float linear_vel, angular_vel ;                     // Variables to store linear and angular velocities.

    public:
        SensorEnable(ros::NodeHandle* nodehandle);                  // Constructor.
        void NameCallBack(const std_msgs::String& msg);             // Callback function for receiving the Robot's Name.
        void KeyboardCallBack(const webots_ros::Int32Stamped& msg); // Callback function for Keyboard Input.
        void CmdvelCallBack(const geometry_msgs::Twist& msg);       // Callback function for receiving cmd_vel Messages.
        void Initialize_sensors();                                  // Function for Initializing Sensors.
        void teleop(int);                                           // Function for TeleOperation.
};

#endif
