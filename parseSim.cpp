#include "parseSim.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <thread>

ParseSim* ParseSim::m_pInstance = nullptr;

void ParseSim::parse(const std::string& sFileName) {
    std::ifstream ifs(sFileName, std::ios::binary | std::ios::in);
    char cType = '9';
    bool flag = true;
    while (!ifs.eof()) {
        ifs.read((char*)&cType, sizeof(char) * 1);
        uint64_t timestamp = 0;
        ifs.read((char*)&timestamp, sizeof(char) * 8);
        //std::this_thread::sleep_for(std::chrono::seconds(1));

        //std::cout << "cType " << cType << " " << timestamp << std::endl;

        switch (cType)
        {
        case '0': // IMU
        {
            sensor_msgs::Imu::ConstPtr msg = std::make_shared<sensor_msgs::Imu::StImu>();
            
            for (int i = 0; i < 6; ++i) {
                ifs.read((char*)&(m_imuData[i]), sizeof(char) * 4);
            }

            msg->header.stamp.sec = timestamp / 1000000000;
            msg->header.stamp.nsec = timestamp % 1000000000;
            msg->angular_velocity.x() = m_imuData[0];
            msg->angular_velocity.y() = m_imuData[1];
            msg->angular_velocity.z() = m_imuData[2];
            msg->linear_acceleration.x() = m_imuData[3];
            msg->linear_acceleration.y() = m_imuData[4];
            msg->linear_acceleration.z() = m_imuData[5];

            //std::cout << "imu " << m_imuData[0] << " " << m_imuData[1] << " " << m_imuData[2] << " " << m_imuData[3] << " " << m_imuData[4] << " " << m_imuData[5] << std::endl;
            notifyImu(msg);
        }
            
            break;
        case '1': // image
        {
            sensor_msgs::ImageConstPtr msg = std::make_shared<sensor_msgs::Image::StImage>();
            size_t len = 0;
            ifs.read((char*)&len, sizeof(char) * 8);
            char *buffer = new char[len];
            ifs.read(buffer, sizeof(char) * len);
            m_vImgdata.resize(len);
            for (int i = 0; i < len; ++i) {
                m_vImgdata[i] = buffer[i];
            }
            
            msg->imageView = cv::imdecode(m_vImgdata, 1);

            //cv::imshow("img", msg->imageView);
            //cv::waitKey(1000);
        }
            // notifyImage(msg);
            break;
        case '2': // pointcloud
        {
            sensor_msgs::PointCloud2::Ptr msg = std::make_shared<sensor_msgs::PointCloud2::StPointCloud>();
            size_t len = 0;
            float x, y, z;
            ifs.read((char*)&len, sizeof(char) * 8);
            //std::cout << "point " << len << std::endl;
            for (size_t i = 0; i < len; i += 3) {
                ifs.read((char*)&x, sizeof(char) * 4);
                ifs.read((char*)&y, sizeof(char) * 4);
                ifs.read((char*)&z, sizeof(char) * 4);
            }
        }
            break;
        default:
            flag = false;
            break;
        }

        if (!flag) {
            std::cout << "error! unknow type" << std::endl;
            break;
        }
    }
    ifs.close();
    std::cout << "parse end" << std::endl;
}