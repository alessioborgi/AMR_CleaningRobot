#ifndef REMAPPING_H
#define REMAPPING_H

/*

File: remapping.h
Author: @AlessioBorgi
Date: 20-02-2024

Description: This file contains the definition header for the Remapping class. 
             The Remapping class is responsible for subscribing to sensor data related to laser scans 
             and republishing them with remapped values.

*/


// Include necessary libraries
#include "ros/ros.h"
#include "webots_ros/Int32Stamped.h"
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <stdio.h>
#include <iostream>
#include <cmath>

// Define constant for time step
#define TIME_STEP 32

// Define the Remapping class
class Remapping{

    private:
        ros::NodeHandle nh_;                                // NodeHandle for ROS communication.
        std::string robot_name_;                            // Variable to store robot name.
        ros::Subscriber subscribe_name_ , subscribe_laser_; // ROS subscribers for topics.
        ros::Publisher publish_laser_;                      // ROS publisher for laser scan topic.
        sensor_msgs::LaserScan local_scan_msg_;             // LaserScan message for storing remapped values.

    public:
        // Constructor
        Remapping(ros::NodeHandle* nodehandle);
        // Callback functions
        void NameCallBack(const std_msgs::String& msg);         // Callback for /model_name topic.
        void LaserCallBack(const sensor_msgs::LaserScan& msg);  // Callback for laser scan topic.
};

#endif
