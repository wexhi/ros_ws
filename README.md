# RosDisplaySystem
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
>> * 打开终端，输入```roscore```打开ros系统
![QQ截图20230630174009](https://github.com/wexhi/RosDisplaySystem/assets/120765859/6727d4a7-b2ce-4709-accb-125fa22c33b4)
>> * 打开新的终端，进入存放bag包的路径:  
       ```cd ~/xxx/```  
     此处的```xxx```替换为存放all.bag包的文件夹  
>>    ![QQ截图20230630175802](https://github.com/wexhi/RosDisplaySystem/assets/120765859/1f95cd6d-fb19-4b50-8d3b-6c1bbd40ffe0)
>> * 播放all.bag包```rosbag play all.bag```
>>    ![QQ截图20230630181048](https://github.com/wexhi/RosDisplaySystem/assets/120765859/65dc2b28-d6b6-4766-b2fb-f75e23e87a0e)
>> * 打开新的终端，显示当前可订阅的话题```rostopic list```
>>   ![QQ截图20230630181456](https://github.com/wexhi/RosDisplaySystem/assets/120765859/8867de45-847a-460a-8bb2-e275e302d87e)  
>>   找到需要订阅的IMU话题```/imu/data_raw```
>> ### 1-1. 直接通过命令行订阅topic查看数据
>>> * 输入命令```rostopic echo /imu/data_raw``` (需要确保all.bag包没有暂停或结束)
>>>   ![QQ截图20230630182406](https://github.com/wexhi/RosDisplaySystem/assets/120765859/6afc7c2f-a2c0-4391-9967-492adcb08571)
>> ### 1-2. 通过编写回调函数查看数据
>>> * 输入命令，查看话题消息的类型  
>>> ```rostopic info /imu/data_raw```        
>>> ```rosmsg show sensor_msgs/Imu```  
>>>![QQ截图20230630183538](https://github.com/wexhi/RosDisplaySystem/assets/120765859/476e053b-0e97-48ee-8e77-6f94de5e216f)
>>> * 根据话题内容编写回调函数  
>>>   ```void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    std::cout << "Header header" << std::endl;
    std::cout << "  seq:  " << msg->header.seq << std::endl;
    std::cout << "  time stamp:  " << msg->header.stamp << std::endl;
    std::cout << "    frame_id:  " << msg->header.frame_id << std::endl;
    std::cout << " orientation" << std::endl;
    std::cout << "     x: " << msg->orientation.x << std::endl;
    std::cout << "     y: " << msg->orientation.y << std::endl;
    std::cout << "     z: " << msg->orientation.z << std::endl;
    std::cout << "     w: " << msg->orientation.w << std::endl;
    std::cout << "orientation_covariance" << std::endl;
    std::cout << "    ";
    for (int i = 0; i < 9; ++i) {
        std::cout << msg->orientation_covariance[i] << " ";
    }
    std::cout << std::endl;
    std::cout << " angular_velocity" << std::endl;
    std::cout << "     x: " << msg->angular_velocity.x << std::endl;
    std::cout << "     y: " << msg->angular_velocity.y << std::endl;
    std::cout << "     z: " << msg->angular_velocity.z << std::endl;
    std::cout << " angular_velocity_covariance" << std::endl;
    std::cout << "    ";
    for (int i = 0; i < 9; ++i) {
        std::cout << msg->angular_velocity_covariance[i] << " ";
    }
    std::cout << std::endl;
    std::cout << " linear_acceleration" << std::endl;
    std::cout << "     x: " << msg->linear_acceleration.x << std::endl;
    std::cout << "     y: " << msg->linear_acceleration.y << std::endl;
    std::cout << "     z: " << msg->linear_acceleration.z << std::endl;
    std::cout << " linear_acceleration_covariance" << std::endl;
    std::cout << "    ";
    for (int i = 0; i < 9; ++i) {
        std::cout << msg->linear_acceleration_covariance[i] << " ";
    }
    std::cout << std::endl;
    std::cout << "************************************************************" << std::endl;
    return ;
}```

 



