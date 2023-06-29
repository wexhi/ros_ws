#include <iostream>
#include <cstring>
#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"

void callback(const sensor_msgs::ImageConstPtr& ptr)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(ptr, sensor_msgs::image_encodings::TYPE_16UC1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return ;
    }
    cv::imshow("depth_camera", cv_ptr->image);
    cv::waitKey(1);
    return ;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "show_depth_camera");
    ros::NodeHandle nodeHandle;
    image_transport::ImageTransport imageTransport(nodeHandle);
    image_transport::Subscriber subscriber = imageTransport.subscribe("/camera/depth/image_rect_raw", 1000, callback);
    ros::spin();

    return 0;
}