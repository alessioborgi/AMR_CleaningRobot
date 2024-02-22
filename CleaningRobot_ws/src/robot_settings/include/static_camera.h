#ifndef STATIC_CAMERA_H
#define STATIC_CAMERA_H

/*

File: static_camera.h
Author: @AlessioBorgi
Date: 12-02-2024

Description: This file contains the StaticCamera class definition. We have the callbacks of linear sensors and rotary sensors. 
             We define the functions to publish the transformations from linear_link to camera_link together with an helper function 
             to convert from Quaternions to Euler Angles.
             
*/

// Include necessary libraries.
#include "ros/ros.h"
#include "webots_ros/Int32Stamped.h"
#include "webots_ros/Float64Stamped.h"
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>
#include <stdio.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <cmath>

// Define constant for time step.
#define TIME_STEP 32

// Define the StaticCamera class.
class StaticCamera{

    private:
        ros::NodeHandle nh_;                                                     // NodeHandle for ROS communication.
        std::string robot_name_;                                                 // Variable to store robot name.
        ros::Subscriber subscribe_name_ , subscribe_linear_ , subscribe_rotary_; // ROS subscribers for topics.
        float current_rot_x , current_rot_y, current_rot_z, current_rot_w;       // Variables to store current orientation in quaternion.
        float roll_ , pitch_ , yaw_ ;                                            // Variables to store current orientation in Euler angles (RPY angles).

    public:
        // Constructor.
        StaticCamera(ros::NodeHandle* nodehandle);

        // Helper function to convert quaternion to Euler angles.
        void ToEulerAngles(float x, float y , float z , float w );

        // Subscriber functions to enable sensors.
        void getLinear();                                           // Subscriber for Linear Rensor.
        void getRotary();                                           // Subscriber for Rotary Sensor.


        // Callback functions.
        void NameCallBack(const std_msgs::String& msg);             // Callback for /model_name topic.

        void LinearCallBack(const webots_ros::Float64Stamped& msg); // Callback for linear sensor topic.

        void RotaryCallBack(const webots_ros::Float64Stamped& msg); // Callback for rotary sensor topic.

        // Functions to publish Transformations.
        void publish_linear_link(float value);                      // Publish Transformation from linear_link.
        void publish_camera_link(float value);                      // Publish Transformation from camera_link.

};

#endif
