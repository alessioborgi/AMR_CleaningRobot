#include <remapping.h>

/*

File: remapping.cpp
Author: @AlessioBorgi
Date: 20-02-2024

Description: This file contains the implementation for the Remapping Class. 
             This class subscribes to sensor data related to laser scans and republishes them with remapped values.

*/

// Constructor: initializes ROS subscribers and publisher.
Remapping::Remapping(ros::NodeHandle* nodehandle):nh_(*nodehandle){
    subscribe_name_ = nh_.subscribe("/model_name", 1, &Remapping::NameCallBack,this);   // Subscribe to /model_name topic.
    publish_laser_ = nh_.advertise<sensor_msgs::LaserScan>("/LaserScan", 1000);         // Advertise publishing on /LaserScan topic.
}

// Callback for /model_name topic: stores the robot name and subscribes to laser scan topic.
void Remapping::NameCallBack(const std_msgs::String& msg){
    Remapping::robot_name_ = msg.data;                                                  // Store the robot name.
    subscribe_laser_ = nh_.subscribe(robot_name_+"/Lidar/laser_scan/layer0", 1, &Remapping::LaserCallBack,this); // Subscribe to laser scan topic.
}

// Callback for laser scan topic: republishes the laser scan message with remapped values.
void Remapping::LaserCallBack(const sensor_msgs::LaserScan& msg){
    local_scan_msg_ = msg;                            // Store the received laser scan message.
    // local_scan_msg_.header.frame_id = "base_link"; // Optionally change the frame id.
    publish_laser_.publish(local_scan_msg_);          // Publish the laser scan message.
}

// Main function: initializes ROS node, creates Remapping object, and spins ROS.
int main(int argc, char **argv){
    ros::init(argc, argv, "Remapping"); // Initialize ROS node.
    ros::NodeHandle nh;                 // Create ROS node handle.
    Remapping member(&nh);              // Create Remapping object.
    ros::spin();                        // Spin ROS.
    return 0;
}
