#include "airsimManager.h"

#include <stdio.h>
#include <chrono>
#include <thread>
#include <future>
#include <time.h>

#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

std::shared_ptr<AirSimManager> AirSimManager::m_pInstance = nullptr;
const bool g_bWrite = false;
#define PI 3.1415926

void AirSimManager::writeToFile() {
   if (m_qBuffers.empty()) {
       return;
   }
   std::pair<uint32_t, char*> data;
   {
       std::lock_guard<std::mutex> locker(m_writeMutex);
       data = m_qBuffers.front();
       m_qBuffers.pop();
   }
   std::ofstream ofs(m_sOutFile, std::ios::binary | std::ios::app);
   ofs.write(data.second, data.first);
   ofs.close();
   delete[] data.second;
}

void AirSimManager::getImuData() {
    auto imu_data = client.getImuData();

    sensor_msgs::Imu::ConstPtr msg = std::make_shared<sensor_msgs::Imu::StImu>();
    msg->header.stamp.sec = imu_data.time_stamp / 1000000000;
    msg->header.stamp.nsec = imu_data.time_stamp % 1000000000;
    msg->angular_velocity.x() = imu_data.angular_velocity[0];
    msg->angular_velocity.y() = imu_data.angular_velocity[1];
    msg->angular_velocity.z() = imu_data.angular_velocity[2];
    msg->linear_acceleration.x() = imu_data.linear_acceleration[0];
    msg->linear_acceleration.y() = imu_data.linear_acceleration[1];
    msg->linear_acceleration.z() = imu_data.linear_acceleration[2];

    if (g_bWrite) {
        // char name[10] = {0};
        // sprintf(name, "%llu.%09llu.imu", msg->header.stamp.sec, msg->header.stamp.nsec);
        // std::ofstream ofs(name, std::ios_base::app);
        // ofs << msg->header.stamp.toSec() << " " << msg->angular_velocity.x() << " " << msg->angular_velocity.y() << " " << msg->angular_velocity.z() << " " << msg->linear_acceleration.x() << " " << msg->linear_acceleration.y() << " " << msg->linear_acceleration.z() << std::endl;
        // ofs.close();
        char* buffer = new char[33];
        char* pBuffer = buffer;
        *(reinterpret_cast<char*>(pBuffer)) = '0';
        pBuffer += 1;
        *(reinterpret_cast<uint64_t*>(pBuffer)) = imu_data.time_stamp;
        pBuffer += 8;
        *(reinterpret_cast<float*>(pBuffer)) = imu_data.angular_velocity[0];
        pBuffer += 4;
        *(reinterpret_cast<float*>(pBuffer)) = imu_data.angular_velocity[1];
        pBuffer += 4;
        *(reinterpret_cast<float*>(pBuffer)) = imu_data.angular_velocity[2];
        pBuffer += 4;
        *(reinterpret_cast<float*>(pBuffer)) = imu_data.linear_acceleration[0];
        pBuffer += 4;
        *(reinterpret_cast<float*>(pBuffer)) = imu_data.linear_acceleration[1];
        pBuffer += 4;
        *(reinterpret_cast<float*>(pBuffer)) = imu_data.linear_acceleration[2];

        {
            std::lock_guard<std::mutex> locker(m_writeMutex);
            m_qBuffers.push(std::make_pair(33, buffer));
        }
    } else {
        notifyImu(msg);
    }

}

void AirSimManager::getImageData(const std::string &camera_name, const std::string& vehicle_name) {
    typedef ImageCaptureBase::ImageRequest ImageRequest;
    typedef ImageCaptureBase::ImageResponse ImageResponse;
    typedef ImageCaptureBase::ImageType ImageType;
    typedef common_utils::FileSystem FileSystem;

    const std::vector<ImageRequest> request{ ImageRequest(camera_name, ImageType::Scene) };
    const std::vector<ImageResponse>& response = client.simGetImages(request, vehicle_name);
    if (response.size()) {
        for (const ImageResponse& image_info : response) {
            // cv::Mat img(image_info.height, image_info.width, CV_8UC4);
            // cv::imdecode(image_info.image_data_uint8, cv::IMREAD_COLOR, &img);
            //cv::imshow("image", img);
            //cv::waitKey(1);

            sensor_msgs::ImageConstPtr msg = std::make_shared<sensor_msgs::Image::StImage>();
            msg->header.stamp.sec = image_info.time_stamp / 1000000000;
            msg->header.stamp.nsec = image_info.time_stamp % 1000000000;
            msg->imageView = cv::imdecode(image_info.image_data_uint8, cv::IMREAD_COLOR);
            
            if (g_bWrite) {
                // char name[10] = {0};
                // sprintf(name, "%llu.%09llu.jpg", msg->header.stamp.sec, msg->header.stamp.nsec);
                // cv::imwrite(name, img);
                char* buffer = new char[image_info.image_data_uint8.size() * sizeof(uint8_t) + 17];
                char* pBuffer = buffer;
                *(reinterpret_cast<char*>(pBuffer)) = '1';
                pBuffer += 1;
                *(reinterpret_cast<uint64_t*>(pBuffer)) = image_info.time_stamp;
                pBuffer += 8;
                *(reinterpret_cast<uint64_t*>(pBuffer)) = image_info.image_data_uint8.size();
                pBuffer += 8;

                memcpy(pBuffer, image_info.image_data_uint8.data(), image_info.image_data_uint8.size() * sizeof(uint8_t));

                {
                    std::lock_guard<std::mutex> locker(m_writeMutex);
                    m_qBuffers.push(std::make_pair(image_info.image_data_uint8.size() * sizeof(uint8_t) + 17, buffer));
                }
            } else {
                notifyImage(msg);
            }
        }
    }
}

void AirSimManager::getLidarData(const std::string& lidar_name, const std::string& vehicle_name) {
    msr::airlib::LidarData lidarData = client.getLidarData(lidar_name, vehicle_name);

    sensor_msgs::PointCloud2::Ptr msg = std::make_shared<sensor_msgs::PointCloud2::StPointCloud>();
    msg->header.stamp.sec = lidarData.time_stamp / 1000000000;
    msg->header.stamp.nsec = lidarData.time_stamp % 1000000000;

    int len = lidarData.point_cloud.size();
    
    for (int i = 0; i < len; i += 3) {
       //pcl::PointXYZINormal pt(lidarData.point_cloud[i], lidarData.point_cloud[i + 1], lidarData.point_cloud[i + 2]);
       msg->pcl_pc.push_back({ lidarData.point_cloud[i], lidarData.point_cloud[i + 1], lidarData.point_cloud[i + 2] });
    }

    if (g_bWrite) {
        // char name[10] = {0};
        // sprintf(name, "%llu.%09llu.xyz", msg->header.stamp.sec, msg->header.stamp.nsec);
        // std::ofstream ofs(name);
        // for (int i = 0; i < len; i += 3) {
        //     ofs << lidarData.point_cloud[i] << " " << lidarData.point_cloud[i + 1] << " " << lidarData.point_cloud[i + 2] << std::endl;
        // }
        // ofs.close();    
        size_t blen = (len << 2) + 17;
        char* buffer = new char[blen];
        char* pBuffer = buffer;
        *(reinterpret_cast<char*>(pBuffer)) = '2';
        pBuffer += 1;
        *(reinterpret_cast<uint64_t*>(pBuffer)) = lidarData.time_stamp;
        pBuffer += 8;
        *(reinterpret_cast<size_t*>(pBuffer)) = len;
        pBuffer += 8;
        for (size_t i = 0; i < len; ++i) {
            *(reinterpret_cast<float*>(pBuffer)) = lidarData.point_cloud[i];
            pBuffer += 4;
        }
        {
            std::lock_guard<std::mutex> locker(m_writeMutex);
            m_qBuffers.push(std::make_pair(blen, buffer));
        }
    } else {
        notifyPoints(msg);
    }
}

void getGPSData(msr::airlib::MultirotorRpcLibClient& client) {
    auto gps_data = client.getGpsData();
    std::cout << "GPS data \n"
    << "gps_data.time_stamp \t" << gps_data.time_stamp << std::endl
    << "gps_data.gnss.time_utc \t" << gps_data.gnss.time_utc << std::endl
    << "gps_data.gnss.geo_point \t" << gps_data.gnss.geo_point << std::endl
    << "gps_data.gnss.eph \t" << gps_data.gnss.eph << std::endl
    << "gps_data.gnss.epv \t" << gps_data.gnss.epv << std::endl
    << "gps_data.gnss.velocity \t" << gps_data.gnss.velocity << std::endl
    << "gps_data.gnss.fix_type \t" << gps_data.gnss.fix_type << std::endl;

    // char name[10] = {0};
    // sprintf(name, "%05d.gps", m_nFrames);
    // std::ofstream ofs(name);
    // ofs << gps_data.time_stamp << " " << gps_data.gnss.time_utc << " " << gps_data.gnss.geo_point
    //     << gps_data.gnss.eph << " " << gps_data.gnss.epv << " " << gps_data.gnss.velocity << " " << gps_data.gnss.fix_type
    //     << std::endl;
    // ofs.close();
}

void getPosition(msr::airlib::MultirotorRpcLibClient& client) {
    auto position = client.getMultirotorState().getPosition();
    std::cout << "get position " << position.x() << " " << position.y() << " " << position.z() << std::endl;

    // char name[10] = {0};
    // sprintf(name, "%05d.pos", m_nFrames);
    // std::ofstream ofs(name);
    // ofs << position.x() << " " << position.y() << " " << position.z() << std::endl;
    // ofs.close();
}

struct GetDataThread {
    explicit GetDataThread(AirSimManager* manager, int type = 1) {
        pManager = manager;
        mType = type;
    }

    void run() {
        mThread = std::thread([this]() {
            this->threadLoop(this->mExitSignal.get_future());
            });
    }

    bool join() {
        if (mThread.joinable()) {
            mThread.join();
            return true;
        }
        return false;
    }

    bool detach() {
        if (mThread.joinable()) {
            mThread.detach();
            return true;
        }
        return false;
    }

    bool isStoped() {
        return mStoped;
    }

    void stop() {
        mStoped = true;
        mExitSignal.set_value();
    }

private:
    int mType = 0;
    bool mStoped = false;
    std::thread mThread;
    std::promise<void> mExitSignal;
    AirSimManager* pManager = nullptr;

    void threadLoop(std::future<void> exitListener) {
        do {
            switch (mType) {
            case 0:
                pManager->getImuData();
                std::this_thread::sleep_for(std::chrono::milliseconds(30)); // 5
                break;
            case 1:
                pManager->getImageData("front_center");
                std::this_thread::sleep_for(std::chrono::milliseconds(70));
                break;
            case 2:
                pManager->getLidarData();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                break;
            default:
                pManager->writeToFile();
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
                break;
            }
        } while (exitListener.wait_for(std::chrono::microseconds(1)) == std::future_status::timeout);
    }
};

void rotateOrigin(float x, float y, float& nx, float& ny, float theta = 30, bool bCrossWise = true) {
    float sintheta = std::sin(theta * PI / 180.0);
    float costheta = std::cos(theta * PI / 180.0);
    if (bCrossWise) {
        nx =  x * costheta - y * sintheta;
        ny =  x * sintheta + y * costheta;
    }
    else {
        nx =  x * costheta + y * sintheta;
        ny = -x * sintheta + y * costheta;
    }
}

void rotate(float x, float y, float xc, float yc, float& nx, float& ny, float theta = 30, bool bCrossWise = true) {
    float ox = x - xc;
    float oy = y - yc;

    rotateOrigin(ox, oy, nx, ny, theta, bCrossWise);
    
    nx += xc;
    ny += yc;
}

void AirSimManager::run() {
    client.confirmConnection();
    client.enableApiControl(true);
    bool ret = client.armDisarm(true);
    client.takeoffAsync()->waitOnLastTask();
    
    std::vector<GetDataThread> getDataThreads;
    for (int i = 0; i <= 3; ++i) {
        getDataThreads.push_back(GetDataThread(this, i));
    }

    std::cout << "run start " << std::endl;
   
    float velocity = .5f;
    float x = 5, y = -13;
    float xn = 0, yn = 0;
    float xc = 5, yc = -8;
    Vector3r origin = client.getMultirotorState().getPosition();
    float height = origin.z() - 5;
    client.moveToPositionAsync(x, y, height, velocity, Utils::max<float>(), DrivetrainType::ForwardOnly, YawMode(false, 0))->waitOnLastTask();

    for (auto& t : getDataThreads) {
        t.run();
    }

    auto rotate2Yaw = [&](float theta = 30, bool crosswise = true) {
        //client.rotateToYawAsync(crosswise ? 90 : -90)->waitOnLastTask();
        client.rotateToYawAsync(crosswise ? theta : -theta)->waitOnLastTask();
        };

    try {
        float step_angle = 10.f;
        float yaw = 0;
        while (yaw < 360.f) {
            yaw += step_angle;
            rotate(x, y, xc, yc, xn, yn, step_angle);
            x = xn;
            y = yn;
            client.moveToPositionAsync(xn, yn, height, velocity, Utils::max<float>(), DrivetrainType::ForwardOnly, YawMode(false, 0))->waitOnLastTask();
        }
    }
    catch (rpc::rpc_error& e) {
        std::cout << "Exception " << e.get_error().as<std::string>() << std::endl;
    }

    for (auto& t : getDataThreads) {
        t.stop();
    }
    for (auto& t : getDataThreads) {
        t.join();
    }

    client.landAsync()->waitOnLastTask();
    client.armDisarm(false);
    client.enableApiControl(false);

    std::cout << "control end " << std::endl;
}

void AirSimManager::getPosition() {
    std::cout << "start get position" << std::endl;
    client.confirmConnection();
    std::cout << "connection done" << std::endl;
    // get control
    client.enableApiControl(false);
    std::cout << "close api control" << std::endl;
    client.enableApiControl(true);
    std::cout << "enable api control" << std::endl;

    // unlock
    bool ret = client.armDisarm(true);
    Vector3r origin = client.getMultirotorState().getPosition();
    std::cout << "current position " << origin.x() << " " << origin.y() << " " << origin.z() << std::endl;
    client.armDisarm(false);

    client.enableApiControl(false);
    std::cout << "get position end " << std::endl;
}

void AirSimManager::test() {
    std::cout << "start get position" << std::endl;
    client.confirmConnection();
    std::cout << "connection done" << std::endl;
    // get control
    client.enableApiControl(false);
    client.enableApiControl(true);
    std::cout << "enable api control" << std::endl;

    // unlock
    bool ret = client.armDisarm(true);
    std::cout << "armDisarm " << ret << std::endl;
    client.takeoffAsync()->waitOnLastTask();
    std::cout << "hover" << std::endl;
    client.rotateToYawAsync(90)->waitOnLastTask();
    std::cout << "test " << std::endl;
    client.armDisarm(false);

    client.enableApiControl(false);
    std::cout << "get position end " << std::endl;
}