#include <iostream>
#include <thread>
#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{
    std::cout << "resolution: " << map_msg->info.resolution << std::endl;
    std::cout << "width: " << map_msg->info.width << std::endl;
    std::cout << "height: " << map_msg->info.height << std::endl;

    double scale = 1.0;
    int width = 1200;
    int height = 1200;
    cv::Point offset = {-1600, -1600};
    cv::Mat map = cv::Mat::zeros(cv::Size(width, height), CV_8UC3);

    for (int i = 0; i < map_msg->info.width * map_msg->info.height; ++i) {
        int x = (i % map_msg->info.width + offset.x) * scale;
        int y = (i / map_msg->info.width + offset.y) * scale;

        if (map_msg->data[i] == -1) {
            cv::circle(map, cv::Point(x, y), 1, cv::Scalar(128,128,128), -1);
        } else if (map_msg->data[i] >= 80) {
            cv::circle(map, cv::Point(x, y), 3, cv::Scalar(0, 0, 255), -1);
        } else {
            cv::circle(map, cv::Point(x, y), 3, cv::Scalar(255, 0, 0), -1);
        }
    }

    cv::imshow("2Dmap", map);
    cv::waitKey(1000);
}

void launchGMapping()
{
    system("rosrun gmapping slam_gmapping");
}

int main(int argc, char** argv)
{
    std::thread gmThread(launchGMapping);

    ros::init(argc, argv, "show_map");
    ros::NodeHandle nodeHandle;
    ros::Subscriber mapSubscriber = nodeHandle.subscribe("/map", 1000, mapCallback);
    ros::spin();

    gmThread.join();  // Wait for the GMapping thread to finish

    return 0;
}
