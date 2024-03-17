#include "airsimManager.h"

#include <stdio.h>
#include <chrono>
#include <thread>
#include <future>
#include <time.h>

#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

std::shared_ptr<AirSimManager> AirSimManager::instance = nullptr;

void AirSimManager::getImuData() {
    auto imu_data = client.getImuData();

    sensor_msgs::Imu::ConstPtr msg = std::make_shared<sensor_msgs::Imu::StImu>();
    msg->header.stamp.stamp = imu_data.time_stamp;
    msg->angular_velocity.x() = imu_data.angular_velocity[0];
    msg->angular_velocity.y() = imu_data.angular_velocity[1];
    msg->angular_velocity.z() = imu_data.angular_velocity[2];
    msg->linear_acceleration.x() = imu_data.linear_acceleration[0];
    msg->linear_acceleration.y() = imu_data.linear_acceleration[1];
    msg->linear_acceleration.z() = imu_data.linear_acceleration[2];
    //ofs << msg->header.stamp.toSec() << " " << msg->angular_velocity.x() << " " << msg->angular_velocity.y() << " " << msg->angular_velocity.z() << " " << msg->linear_acceleration.x() << " " << msg->linear_acceleration.y() << " " << msg->linear_acceleration.z() << std::endl;
    notifyImu(msg);
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
            cv::Mat img(image_info.height, image_info.width, CV_8UC4);
            cv::imdecode(image_info.image_data_uint8, cv::IMREAD_COLOR, &img);
            //cv::imshow("image", img);
            //cv::waitKey(1);

            sensor_msgs::ImageConstPtr msg = std::make_shared<sensor_msgs::Image::StImage>();
            msg->header.stamp.sec = image_info.time_stamp;
            msg->imageView = cv::imdecode(image_info.image_data_uint8, cv::IMREAD_COLOR);
            notifyImage(msg);
        }
    }
}

void AirSimManager::getLidarData(const std::string& lidar_name, const std::string& vehicle_name) {
    msr::airlib::LidarData lidarData = client.getLidarData(lidar_name, vehicle_name);

    sensor_msgs::PointCloud2::Ptr msg = std::make_shared<sensor_msgs::PointCloud2::StPointCloud>();
    int len = lidarData.point_cloud.size();
    
    for (int i = 0; i < len; i += 3) {
        //pcl::PointXYZINormal pt(lidarData.point_cloud[i], lidarData.point_cloud[i + 1], lidarData.point_cloud[i + 2]);
        msg->pcl_pc.push_back({ lidarData.point_cloud[i], lidarData.point_cloud[i + 1], lidarData.point_cloud[i + 2] });
    }
    notifyPoints(msg);
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
}

void getPosition(msr::airlib::MultirotorRpcLibClient& client) {
    auto position = client.getMultirotorState().getPosition();
    std::cout << "get position " << position.x() << " " << position.y() << " " << position.z() << std::endl;
}

struct GetDataThread {
    explicit GetDataThread(AirSimManager* manager) {
        pManager = manager;
    }

    void run() {
        mThread = std::thread([this]() {
            this->threadLoop(this->mExitSignal.get_future());
            });
        mThread.detach();
    }

    void join() {
        mThread = std::thread([this]() {
            this->threadLoop(this->mExitSignal.get_future());
            });
        mThread.join();
    }

    void stop() {
        mExitSignal.set_value();
    }

private:
    std::thread mThread;
    std::promise<void> mExitSignal;
    AirSimManager* pManager = nullptr;
    void threadLoop(std::future<void> exitListener) {
        do {
            pManager->getImageData();
            pManager->getImuData();
            pManager->getLidarData();

            std::this_thread::sleep_for(std::chrono::milliseconds(10000));
        } while (exitListener.wait_for(std::chrono::microseconds(1)) == std::future_status::timeout);
    }
};

void AirSimManager::run() {
    client.confirmConnection();
    // get control
    client.enableApiControl(true);

    GetDataThread getDataThread(this);

    std::thread runThread([this, &getDataThread]() {
        
        // unlock
        bool ret = client.armDisarm(true);
        // wait for last task to finish
        client.takeoffAsync(5)->waitOnLastTask();

        try {
            Vector3r origin = client.getMultirotorState().getPosition();
            std::cout << "current position " << origin.x() << " " << origin.y() << " " << origin.z() << std::endl;
            float height = origin.z();

            float velocity = 1.f;
            vector<Vector3r> path = { Vector3r(10, 0, height), Vector3r(10, 10, height), Vector3r(-10, 10, height), Vector3r(-10, -10, height), Vector3r(10, -10, height), Vector3r(10, 0, height), Vector3r(0, 0, height) };

            client.moveOnPathAsync(path, velocity, Utils::max<float>(), DrivetrainType::ForwardOnly, YawMode(false, 0))->waitOnLastTask();
            getDataThread.stop();
            std::cout << "move to path " << std::endl;
        }
        catch (rpc::rpc_error& e) {
            std::cout << "Exception " << e.get_error().as<std::string>() << std::endl;
        }
        client.landAsync()->waitOnLastTask();
        client.armDisarm(false);
        client.enableApiControl(false);
        std::cout << "control end " << std::endl;
    });
    //runThread.detach();

    getDataThread.join();
}