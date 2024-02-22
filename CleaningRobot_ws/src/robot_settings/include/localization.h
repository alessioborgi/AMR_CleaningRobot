#ifndef LOCALIZATION_H
#define LOCALIZATION_H

/*

File: localization.h
Author: @AlessioBorgi
Date: 15-02-2024

Description: This file contains the definition header for the Localization class. 
             The Localization class is responsible for subscribing to sensor data related to localization,
             such as GPS and IMU data. It also handles the transformation broadcasting for localization data.

*/

// Include necessary libraries,
#include "ros/ros.h"
#include "webots_ros/Int32Stamped.h"
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/robot_get_device_list.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>
#include <stdio.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <cmath>

// Define constant for time step,
#define TIME_STEP 32

// Define the Localization class,
class Localization{

    private:
        ros::NodeHandle nh_;                                                // NodeHandle for ROS communication.
        std::string robot_name_;                                            // Variable to store robot name.
        ros::Subscriber subscribe_name_ , subscribe_gps_ , subscribe_imu_;  // ROS subscribers for topics.
        std::vector<ros::ServiceClient> vec_velocity_;                      // Vector to store service clients.
        float current_x , current_y, current_z;                             // Variables to store current position.
        float current_rot_x , current_rot_y, current_rot_z, current_rot_w;  // Variables to store current orientation in quaternion.
        float roll_ , pitch_ , yaw_ ;                                       // Variables to store current orientation in Euler angles.

    public:
        // Constructor.
        Localization(ros::NodeHandle* nodehandle);

        // Helper function to convert quaternion to Euler angles
        void ToEulerAngles(float x, float y , float z , float w );
        
        // Callback functions.
        void NameCallBack(const std_msgs::String& msg);             // Callback for /model_name topic
        void GPSCallBack(const geometry_msgs::PointStamped& msg);   // Callback for GPS sensor topic
        void IMUCallBack(const sensor_msgs::Imu& msg);              // Callback for IMU sensor topic

        // Subscriber functions to enable sensors.
        void getIMU();                                              // Subscriber for IMU sensor
        void getGPS();                                              // Subscriber for GPS sensor

        // Functions to publish transformations.
        void publish_base_link();                                   // Publish transformation for base_link.
        void publish_lidar_link();                                  // Publish transformation for lidar_link.

};

#endif
