# RosDisplaySystem
# https://www.bilibili.com/video/BV11X4y1n7s6/
# 任务内容
1、利用松灵小车上录制的bag包数据（从QQ群下载），编写软件，展示ROS系统中的各类数据，具体包括：

2、用命令行窗口显示小车的IMU和里程计（odometry）数据；

3、用图形界面显示颜色相机和深度相机的数据（利用OpenCV库）；

4、用图形界面显示激光雷达的点云数据（利用PCL库）；

5、自行选择一种高级算法（例如语义分割、三维重建、导航定位（SLAM）等），实现该算法（可以直接利用第三方库），将其集成到系统中。
# 代码要求
所有程序代码采用C++编写，使用git进行源代码管理；

类名、变量名、函数名应符合C++的命名规范，并在代码中前后保持一致；

涉及面向对象的程序，例如自定义的类，应符合面向对象的设计原则；

正确使用头文件和源文件，自定义的头文件应符合头文件的编写原则，例如用条件宏定义确保头文件不被多次引用、不在头文件中进行类和函数的实现（模板除外）；

项目必须是ROS项目，符合ROS的项目的规范，正确编写CmakeLists.txt等文件；

# 提交资料
程序源代码，需要包含git仓库（.git文件夹），源代码需上传到公网上的git仓库，并提供仓库的URL地址
录制程序的演示视频，视频可以传到B站，上传视频播放的URL地址；
提交一份报告，描述程序实现的关键步骤、算法和结果。
# 成绩构成
代码规范性占10分；
git的使用占10分；
报告和视频占20分；
任务1-3占50分；
任务4占10分。

# ROS数据展示
# 一、显示小车的IMU和里程计（odometry）数据
> ## 1. 显示IMU数据
>> * 打开终端，输入
>>   ```bash
>>   roscore
>>   ```
>>   打开ros系统
![QQ截图20230630174009](https://github.com/wexhi/RosDisplaySystem/blob/master/image/1.png)
>> * 打开新的终端，进入存放bag包的路径:  
       ```cd ~/xxx/```  
     此处的```xxx```替换为存放all.bag包的文件夹  
>>    ![QQ截图20230630175802](https://github.com/wexhi/RosDisplaySystem/blob/master/image/2.png)
>> * 播放all.bag包
>>  ```bash
>>   rosbag play all.bag
>>  ```
>>    ![QQ截图20230630181048](https://github.com/wexhi/RosDisplaySystem/blob/master/image/3.png)
>> * 打开新的终端，显示当前可订阅的话题
>>   ```bash
>>   rostopic list
>>   ```
>>   ![QQ截图20230630181456](https://github.com/wexhi/RosDisplaySystem/blob/master/image/4.png)  
>>   找到需要订阅的IMU话题```/imu/data_raw```
>> ### 1-1. 直接通过命令行订阅topic查看数据
>>> * 输入命令
>>>   ```bash
>>>   rostopic echo /imu/data_raw
>>>   ```
>>>   (需要确保all.bag包没有暂停或结束)
>>>   ![QQ截图20230630182406](https://github.com/wexhi/RosDisplaySystem/blob/master/image/5.png)
>> ### 1-2. 通过编写回调函数查看数据
>>> * 输入命令，查看话题消息的类型  
>>> ```bash
>>> rostopic info /imu/data_raw        
>>> rosmsg show sensor_msgs/Imu
>>> ```  
>>>![QQ截图20230630183538](https://github.com/wexhi/RosDisplaySystem/blob/master/image/6.png)
>>> * 根据话题内容编写回调函数  
>>>  ```cpp
>>>   void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
>>>{
>>>    std::cout << "Header header" << std::endl;
>>>    std::cout << "  seq:  " << msg->header.seq << std::endl;
>>>    std::cout << "  time stamp:  " << msg->header.stamp << std::endl;
>>>    std::cout << "    frame_id:  " << msg->header.frame_id << std::endl;
>>>    std::cout << " orientation" << std::endl;
>>>    std::cout << "     x: " << msg->orientation.x << std::endl;
>>>    std::cout << "     y: " << msg->orientation.y << std::endl;
>>>    std::cout << "     z: " << msg->orientation.z << std::endl;
>>>    std::cout << "     w: " << msg->orientation.w << std::endl;
>>>    std::cout << "orientation_covariance" << std::endl;
>>>    std::cout << "    ";
>>>    for (int i = 0; i < 9; ++i) {
>>>        std::cout << msg->orientation_covariance[i] << " ";
>>>    }
>>>    std::cout << std::endl;
>>>    std::cout << " angular_velocity" << std::endl;
>>>    std::cout << "     x: " << msg->angular_velocity.x << std::endl;
>>>    std::cout << "     y: " << msg->angular_velocity.y << std::endl;
>>>    std::cout << "     z: " << msg->angular_velocity.z << std::endl;
>>>    std::cout << " angular_velocity_covariance" << std::endl;
>>>    std::cout << "    ";
>>>    for (int i = 0; i < 9; ++i) {
>>>        std::cout << msg->angular_velocity_covariance[i] << " ";
>>>    }
>>>    std::cout << std::endl;
>>>    std::cout << " linear_acceleration" << std::endl;
>>>    std::cout << "     x: " << msg->linear_acceleration.x << std::endl;
>>>    std::cout << "     y: " << msg->linear_acceleration.y << std::endl;
>>>    std::cout << "     z: " << msg->linear_acceleration.z << std::endl;
>>>    std::cout << " linear_acceleration_covariance" << std::endl;
>>>    std::cout << "    ";
>>>    for (int i = 0; i < 9; ++i) {
>>>        std::cout << msg->linear_acceleration_covariance[i] << " ";
>>>    }
>>>    std::cout << std::endl;
>>>    std::cout << "************************************************************" << std::endl;
>>>    return ;
>>>}
>>> ```     
>>> * 订阅IMU话题
>>> ```cpp
>>> int main(int argc, char **argv)
>>>{
>>>    ros::init(argc, argv, "show_imu");
>>>    ros::NodeHandle nh;
>>>    ros::Subscriber imu_sub = nh.subscribe("/imu/data_raw", 1000, imuCallback);
>>>    ros::spin();
>>>    return 0;
>>>}
>>> ```
>>> * 运行程序
>>>   在进入代码的工作空间，更新源，随后运行show_imu程序
>>>   ```bash
>>>   cd ros_ws/
>>>   source ./devel/setup.bash
>>>   rosrun show_imu show_imu
>>>   ```
>>>   ![QQ截图20230630192059](https://github.com/wexhi/RosDisplaySystem/blob/master/image/7.png)
> ## 2. 显示里程计（odometry）数据
>> ### 2-1. 通过命令行查看数据
>>> * 直接在终端中输入
>>>   ```bash
>>>   rostopic echo /odom
>>>   ```
>>>   ![QQ截图20230630193155](https://github.com/wexhi/RosDisplaySystem/blob/master/image/8.png)
>> ### 2-2. 通过回调函数查看数据
>> 根据显示数据编写回调函数
```cpp 
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
```
>>> * 订阅里程计话题
```cpp
int main(int argc, char** argv)
{
    ros::init(argc, argv, "show_odometry");
    ros::NodeHandle nodeHandle;
    ros::Subscriber subscriber = nodeHandle.subscribe("/odom", 1000, callback);
    ros::spin();
    return 0;
}
```
>>> * 运行show_odom
>>>   ```bash
>>>   rosrun show_odom show_odom
>>>   ```
>>>   ![QQ截图20230630194325](https://github.com/wexhi/RosDisplaySystem/blob/master/image/9.png)
# 二、用图形界面显示颜色相机和深度相机的数据（利用OpenCV库）
> 1. 显示color相机的数据
>>    * 查看话题和消息的具体内容
>> ```bash
>> rostopic info /camera/color/camera_info        
>> rosmsg show sensor_msgs/CameraInfo
>> ```
>> ![QQ截图20230630195000](https://github.com/wexhi/RosDisplaySystem/blob/master/image/10.png)
>>    * 编写回调函数
```cpp
void cameraCallback(const sensor_msgs::ImageConstPtr& ptr)
{
    cv_bridge::CvImagePtr cvPtr;
    try {
        cvPtr = cv_bridge::toCvCopy(ptr, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::imshow("color_camera", cvPtr->image);
    cv::waitKey(1);
    return;
}
```
>>    * 订阅话题内容
```cpp
int main(int argc, char** argv)
{
    ros::init(argc, argv, "show_color_camera");
    ros::NodeHandle nodeHandle;
    image_transport::ImageTransport imageTransport(nodeHandle);
    image_transport::Subscriber subscriber = imageTransport.subscribe("/camera/color/image_raw", 1000, cameraCallback);
    ros::spin();

    return 0;
}
```
>>    * 在工作空间中输入
>> ```bash
>>      rosrun show_RGB_camera show_RGB_camera
>> ```  
>>    * 运行show_RGB_camera  
>>     ![QQ截图20230630195809](https://github.com/wexhi/RosDisplaySystem/blob/master/image/11.png)
> 2. 显示深度相机的数据
>> * 修改代码
```cpp
 void callback(const sensor_msgs::ImageConstPtr& ptr)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(ptr, sensor_msgs::image_encodings::TYPE_16UC1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::imshow("depth_camera", cv_ptr->image);
    cv::waitKey(1);
}
```
>> * 运行程序
>> ```bash
>>      rosrun show_depth_camera show_depth_camera
>> ```
>> ![QQ截图20230630200711](https://github.com/wexhi/RosDisplaySystem/blob/master/image/12.png)
# 三、用图形界面显示激光雷达的点云数据（利用PCL库）
> * 查看话题消息的类型
> ```bash
> rostopic info /rslidar_points
> ```
> * 编写回调函数
```cpp
void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*msg, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, cloud);
    viewer.showCloud(cloud.makeShared());
}
```
> * 运行程序显示点云图
> ```bash
> rosrun show_lidar_points show_lidar_points
> ```
> ![QQ截图20230630201539](https://github.com/wexhi/RosDisplaySystem/blob/master/image/14.png)
# 四、二维重建地图
## 1. 通过编写回调函数显示地图
> * 启动gmapping
```cpp
void launchGMapping()
{
    system("rosrun gmapping slam_gmapping");
}
```
> * 利用opencv绘制2D地图
```cpp
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
```
> * 显示地图  
> ![QQ截图20230630201539](https://github.com/wexhi/RosDisplaySystem/blob/master/image/19.png)
## 2.利用rviz显示地图
> * 重新播放all.bag
>   ```bash
>   rosbag play --pause all.bag
>   ```
>   > * 开启新终端，输入
>   ```bash
>   rosrun gmapping slam_gmapping scan
>   ```
> * 打开rviz
>   ```bash
>   rviz
>   ```
> ![QQ截图20230630201539](https://github.com/wexhi/RosDisplaySystem/blob/master/image/15.png)
> * 点击左下方Add选项  
> ![QQ截图20230630201539](https://github.com/wexhi/RosDisplaySystem/blob/master/image/16.png)
> * 将Topic选择为/map   
> ![QQ截图20230630201539](https://github.com/wexhi/RosDisplaySystem/blob/master/image/17.png)  
>   此时可以看到重建后的二维地图  
> ![QQ截图20230630201539](https://github.com/wexhi/RosDisplaySystem/blob/master/image/18.png)






   

 



