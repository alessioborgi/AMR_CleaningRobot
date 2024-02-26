// Include the header file for the StaticCamera class
#include <static_camera.h>

/*

File: static_camera.cpp
Author: @AlessioBorgi
Date: 12-02-2024

Description: This file contains the implementation for the Camera Class. This will take the name of the robot, it will perform 
             an initialization of all the sensors. Moreover, here we define the TeleOperation through Keyboard for handling mechanically 
             the movements of the robot.

*/

// Constructor for the StaticCamera class.
StaticCamera::StaticCamera(ros::NodeHandle* nodehandle):nh_(*nodehandle){

    // Subscribe to the "/model_name" topic to get the robot name.
    subscribe_name_ = nh_.subscribe("/model_name", 1, &StaticCamera::NameCallBack,this);
}

// Callback function for "/model_name" topic.
void StaticCamera::NameCallBack(const std_msgs::String& msg){

    // Store the received robot name.
    StaticCamera::robot_name_ = msg.data;

    // Call functions to subscribe to other sensors with the robot name.
    getLinear();
    getRotary();
}

// Function to publish transformation from linear_link.
void StaticCamera::publish_linear_link(float value){

    // Declare the TransformBroadcaster and the Transform variables.
    static tf::TransformBroadcaster br;
    tf::Transform transform;

    // Set position of linear_link.
    transform.setOrigin( tf::Vector3(0,0,value+0.05/2)); //Robot height/2 offset for linear link
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);

    // Broadcast Transformation from the base_link(box) to the linear_link(camera's link).
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link" , "linear_link"));
}

// Function to publish transformation from camera_link.
void StaticCamera::publish_camera_link(float value){

    // Declare the TransformBroadcaster and the Transform variables.
    static tf::TransformBroadcaster br;
    tf::Transform transform;

    // Set position of camera_link.
    transform.setOrigin( tf::Vector3(0,0,0));
    tf::Quaternion q;
    q.setRPY(0, 0 ,value + 1.57 ); // 90 degree off camera in proto.
    transform.setRotation(q);

    // Broadcast transformation from rotary_link to camera_link.
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "rotary_link" , "camera_link"));
}


// Subscribe to Linear Sensor topic.
void StaticCamera::getLinear(){

    // Subscribe to the Linear Sensor Topic.
    subscribe_linear_ = nh_.subscribe(robot_name_+"/Linear_sensor/value", 1, &StaticCamera::LinearCallBack,this);
}

// Subscribe to Rotary Sensor topic.
void StaticCamera::getRotary(){

    // Subscribe to the Rotation Sensor Topic.
    subscribe_rotary_ = nh_.subscribe(robot_name_+"/Rotation_sensor/value", 1, &StaticCamera::RotaryCallBack,this);
}

// Callback function for linear sensor topic.
void StaticCamera::LinearCallBack(const webots_ros::Float64Stamped& msg){

    // Publish Transformation from linear_link. 
    publish_linear_link(msg.data);
}

// Callback function for rotary sensor topic.
void StaticCamera::RotaryCallBack(const webots_ros::Float64Stamped& msg){

    // Publish Transformation from camera_link.
    publish_camera_link(msg.data);
}

// Function to convert quaternion to Euler angles.
void StaticCamera::ToEulerAngles(float x, float y , float z , float w ) {

    // Calculate Roll.
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    roll_ = std::atan2(sinr_cosp, cosr_cosp);

    // Calculate Pitch.
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        pitch_ = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch_ = std::asin(sinp);

    // Calculate Yaw.
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw_ = std::atan2(siny_cosp, cosy_cosp);
}

// Main function
int main(int argc, char **argv){

    // Initialize ROS node.
    ros::init(argc, argv, "static_camera");

    // Create a NodeHandle object.
    ros::NodeHandle nh;

    // Create an instance of the StaticCamera class.
    StaticCamera member(&nh);

    // Enter a loop to process ROS callbacks until the node is shut down.
    ros::spin();

    // Return 0 to indicate successful completion of the program.
    return 0;
}
