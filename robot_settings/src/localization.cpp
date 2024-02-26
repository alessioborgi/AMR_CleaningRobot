#include <localization.h> 
/*

File: localization.cpp
Author: @AlessioBorgi
Date: 10-02-2024

Description: This file contains the implementation for the Localization Class. 
             This class subscribes to sensor data related to localization, such as GPS and IMU data.
             It also handles the transformation broadcasting for localization data.

*/


// Constructor for the Localization class.
Localization::Localization(ros::NodeHandle* nodehandle):nh_(*nodehandle){

    // Subscribe to the /model_name topic to get the robot name.
    subscribe_name_ = nh_.subscribe("/model_name", 1, &Localization::NameCallBack,this);
}

// Function to publish transformation for base_link.
void Localization::publish_base_link(){

    // Create the TransformBoradcaster and the Transform. 
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(-current_x, current_z, current_y) ); // Set position based on GPS data.

    // Convert quaternion to Euler angles and set orientation.
    ToEulerAngles(current_rot_x,current_rot_y,current_rot_z,current_rot_w);
    tf::Quaternion q;
    q.setRPY(0, 0, roll_);
    transform.setRotation(q);

    // Broadcast transformation from the map to base_link.
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
}

// Function to publish transformation for lidar_link.
void Localization::publish_lidar_link(){

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0,0,0));
  tf::Quaternion q;
  q.setRPY(0, 3.142, 0);            // Set orientation (yaw 180 degrees)
  transform.setRotation(q);

  // Broadcast transformation from base_link to the Lidar topic.
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link" , robot_name_+"/Lidar"));
}

// Subscriber function for GPS sensor.
void Localization::getGPS(){
    
    // Take the current Position Data from the GPS sensor and subscribe to the .../global/values Topic. 
    subscribe_gps_ = nh_.subscribe(robot_name_+"/global/values", 1, &Localization::GPSCallBack,this);
}

// Subscriber function for IMU sensor.
void Localization::getIMU(){

    // Take the current Rotational Quaternion Data from the IMU sensor and subscribe to the .../IMU/quaternion Topic.
    subscribe_imu_ = nh_.subscribe(robot_name_+"/IMU/quaternion", 1, &Localization::IMUCallBack,this);
}

// Callback function for /model_name topic.
void Localization::NameCallBack(const std_msgs::String& msg){

    // Store the received robot name.
    Localization::robot_name_ = msg.data;

    // Call functions to subscribe to GPS and IMU sensors
    getGPS();
    getIMU();
}

// Callback function for GPS sensor topic.
void Localization::GPSCallBack(const geometry_msgs::PointStamped& msg){

    // Store current position data from GPS sensor.
    current_x = msg.point.x;
    current_y = msg.point.y;
    current_z = msg.point.z;
}

// Callback function for IMU sensor topic
void Localization::IMUCallBack(const sensor_msgs::Imu& msg){

    // Store current orientation data from IMU sensor.
    current_rot_x = msg.orientation.x;
    current_rot_y = msg.orientation.y;
    current_rot_z = msg.orientation.z;
    current_rot_w = msg.orientation.w;

    // Publish transformations for base_link and lidar_link.
    publish_base_link();
    publish_lidar_link();
}


// Function to convert quaternion to Euler angles.
void Localization::ToEulerAngles(float x, float y , float z , float w ) {

    // Calculate roll, pitch, and yaw.

    // Roll (x-axis rotation).
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    roll_ = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation).
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        pitch_ = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch_ = std::asin(sinp);

    // Yaw (z-axis rotation).
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw_ = std::atan2(siny_cosp, cosy_cosp);
}

// Main function
int main(int argc, char **argv){

    // Initialize ROS node.
    ros::init(argc, argv, "localization");

    // Create a NodeHandle object.
    ros::NodeHandle nh;

    // Create an instance of the Localization class.
    Localization member(&nh);

    // Enter a loop to process ROS callbacks until the node is shut down.
    ros::spin();
    
    // Return 0 to indicate successful completion of the program.
    return 0;
}
