#include "ros/ros.h"
#include<iostream>
#include<sensor_msgs/Imu.h>

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    std::cout << "std_msgs/Header header" << std::endl;
    std::cout << "    uint32 seq: " << msg->header.seq << std::endl;
    std::cout << "    time stamp: " << msg->header.stamp << std::endl;
    std::cout << "    string frame_id: " << msg->header.frame_id << std::endl;
    std::cout << "geometry_msgs/Quaternion orientation" << std::endl;
    std::cout << "    float64 x: " << msg->orientation.x << std::endl;
    std::cout << "    float64 y: " << msg->orientation.y << std::endl;
    std::cout << "    float64 z: " << msg->orientation.z << std::endl;
    std::cout << "    float64 w: " << msg->orientation.w << std::endl;
    std::cout << "float64[9] orientation_covariance" << std::endl;
    std::cout << "    ";
    for (int i = 0; i < 9; ++i) {
        std::cout << msg->orientation_covariance[i] << " ";
    }
    std::cout << std::endl;
    std::cout << "geometry_msgs/Vector3 angular_velocity" << std::endl;
    std::cout << "    float64 x: " << msg->angular_velocity.x << std::endl;
    std::cout << "    float64 y: " << msg->angular_velocity.y << std::endl;
    std::cout << "    float64 z: " << msg->angular_velocity.z << std::endl;
    std::cout << "float64[9] angular_velocity_covariance" << std::endl;
    std::cout << "    ";
    for (int i = 0; i < 9; ++i) {
        std::cout << msg->angular_velocity_covariance[i] << " ";
    }
    std::cout << std::endl;
    std::cout << "geometry_msgs/Vector3 linear_acceleration" << std::endl;
    std::cout << "    float64 x: " << msg->linear_acceleration.x << std::endl;
    std::cout << "    float64 y: " << msg->linear_acceleration.y << std::endl;
    std::cout << "    float64 z: " << msg->linear_acceleration.z << std::endl;
    std::cout << "float64[9] linear_acceleration_covariance" << std::endl;
    std::cout << "    ";
    for (int i = 0; i < 9; ++i) {
        std::cout << msg->linear_acceleration_covariance[i] << " ";
    }
    std::cout << std::endl;
    std::cout << "************************************************************" << std::endl;
    return ;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "show_imu");
    ros::NodeHandle nh;
    ros::Subscriber imu_sub = nh.subscribe("/imu/data_raw", 1000, imuCallback);
    ros::spin();
    return 0;
}