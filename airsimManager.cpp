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
    ////std::cout << msg->header.stamp.toSec() << " " << msg->angular_velocity.x() << " " << msg->angular_velocity.y() << " " << msg->angular_velocity.z() << " " << msg->linear_acceleration.x() << " " << msg->linear_acceleration.y() << " " << msg->linear_acceleration.z() << std::endl;
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
            case 1:
                pManager->getImageData();
                std::this_thread::sleep_for(std::chrono::milliseconds(8));
                break;
            case 2:
                pManager->getLidarData();
                std::this_thread::sleep_for(std::chrono::milliseconds(15));
                break;
            default:
                pManager->getImuData();
                break;
            }
        } while (exitListener.wait_for(std::chrono::microseconds(1)) == std::future_status::timeout);
    }
};

void acrosswiseOrigin(float x, float y, float& nx, float& ny, float theta = 30) {
    float sintheta = std::sin(theta / 180.0);
    float costheta = std::cos(theta / 180.0);
    nx = y * costheta - x * sintheta;
    ny = x * sintheta + y * costheta;
}

void acrosswise(float x, float y, float xc, float yc, float& nx, float& ny, float theta = 30) {
    float sintheta = std::sin(theta / 180.0);
    float costheta = std::cos(theta / 180.0);

    float ox = x - xc;
    float oy = y - yc;
    float nox = oy * costheta - ox * sintheta;
    float noy = ox * sintheta + oy * costheta;

    nx = nox + xc;
    ny = noy + yc;
}

void AirSimManager::run() {
    client.confirmConnection();
    client.enableApiControl(true);
    bool ret = client.armDisarm(true);
    client.takeoffAsync()->waitOnLastTask();
    
    std::vector<GetDataThread> getDataThreads;
    for (int i = 0; i < 3; ++i) {
        getDataThreads.push_back(GetDataThread(this, i));
    }

    std::cout << "run start " << std::endl;
   
    for (auto& t : getDataThreads) {
        t.run();
    }
    float velocity = .5f;
    auto rotate = [&](float theta = 30, bool crosswise = true) {
        //client.rotateToYawAsync(crosswise ? 90 : -90)->waitOnLastTask();
        client.rotateToYawAsync(crosswise ? theta : -theta)->waitOnLastTask();
        std::this_thread::sleep_for(std::chrono::seconds(5));
        };

    std::this_thread::sleep_for(std::chrono::seconds(10));
    try {
        Vector3r origin = client.getMultirotorState().getPosition();
        float height = origin.z();
        vector<Vector3r> path = {
            Vector3r(0, 0, height)
        };

        float x = 0, y = 0;
        float xn = 0, yn = 0;
        float xc = 5, yc = 5;

        rotate(90);
        int count = 0;
        float step_angle = 15.f;
        while (count < std::ceil(360 / step_angle)) {
            acrosswise(x, y, xc, yc, xn, yn, step_angle);
            client.moveToPositionAsync(xn, yn, height, velocity, Utils::max<float>(), DrivetrainType::ForwardOnly, YawMode(false, 0))->waitOnLastTask();
            std::this_thread::sleep_for(std::chrono::seconds(5));
            rotate(10);
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }

        for (auto& t : getDataThreads) {
            t.stop();
        }
    }
    catch (rpc::rpc_error& e) {
        std::cout << "Exception " << e.get_error().as<std::string>() << std::endl;
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