#include <iostream>
#include <memory>
#include <cstring>
#include <ros/ros.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

pcl::visualization::CloudViewer viewer("Cloud Viewer");

void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*msg, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, cloud);
    viewer.showCloud(cloud.makeShared());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "show_lidar_points");
    ros::NodeHandle nodeHandle;
    ros::Subscriber subscriber = nodeHandle.subscribe("/rslidar_points", 1000, callback);
    ros::spin();
    return 0;
}
