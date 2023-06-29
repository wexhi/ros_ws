#include "ros/ros.h"
#include <iostream>
#include <nav_msgs/Odometry.h>

void callback(const nav_msgs::Odometry::ConstPtr& ptr)
{
    std::cout << "Header header" << std::endl;
    std::cout << "    seq: " << ptr->header.seq << std::endl;
    std::cout << "    stamp: " << ptr->header.stamp << std::endl;
    std::cout << "    frame_id: " << ptr->header.frame_id << std::endl;
    std::cout << "child_frame_id: " << ptr->child_frame_id << std::endl;
    std::cout << "PoseWithCovariance pose" << std::endl;
    std::cout << "    Pose pose" << std::endl;
    std::cout << "        Point position" << std::endl;
    std::cout << "            x: " << ptr->pose.pose.position.x << std::endl;
    std::cout << "            y: " << ptr->pose.pose.position.y << std::endl;
    std::cout << "            z: " << ptr->pose.pose.position.z << std::endl;
    std::cout << "        Quaternion orientation" << std::endl;
    std::cout << "            x: " << ptr->pose.pose.orientation.x << std::endl;
    std::cout << "            y: " << ptr->pose.pose.orientation.y << std::endl;
    std::cout << "            z: " << ptr->pose.pose.orientation.z << std::endl;
    std::cout << "            w: " << ptr->pose.pose.orientation.w << std::endl;
    std::cout << "    covariance" << std::endl;
    std::cout << "        ";
    for (int i = 0; i < 36; i++) {
        std::cout << ptr->pose.covariance[i] << " ";
    }
    std::cout << std::endl;
    std::cout << "TwistWithCovariance twist" << std::endl;
    std::cout << "    Twist twist" << std::endl;
    std::cout << "        Vector3 linear" << std::endl;
    std::cout << "            x: " << ptr->twist.twist.linear.x << std::endl;
    std::cout << "            y: " << ptr->twist.twist.linear.y << std::endl;
    std::cout << "            z: " << ptr->twist.twist.linear.z << std::endl;
    std::cout << "        Vector3 angular" << std::endl;
    std::cout << "            x: " << ptr->twist.twist.angular.x << std::endl;
    std::cout << "            y: " << ptr->twist.twist.angular.y << std::endl;
    std::cout << "            z: " << ptr->twist.twist.angular.z << std::endl;
    std::cout << "    covariance" << std::endl;
    std::cout << "        ";
    std::cout << "[";
    for (int i = 0; i < 36; i++) {
        if (i < 35)
            std::cout << ptr->twist.covariance[i] << ", ";
        else
            std::cout << ptr->twist.covariance[i];
    }
    std::cout << "]";
    std::cout << std::endl;
    std::cout << "*************************************************************" << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "show_odometry");
    ros::NodeHandle nodeHandle;
    ros::Subscriber subscriber = nodeHandle.subscribe("/odom", 1000, callback);
    ros::spin();
    return 0;
}
