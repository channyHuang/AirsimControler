# AirsimControler
using c++ control AirSim environment in windows without ROS 

场景：使用AirSim模拟，主要为平地+汽车+无人机。

使用c++控制无人机绕场景飞行一周并每间隔一定时间获取场景数据。数据包括IMU、RGB图像和激光雷达。Airsim的激光雷达获取的点云数据只有三维坐标xyz，没有颜色也没有ring信息，rgb颜色需要结合图像计算。