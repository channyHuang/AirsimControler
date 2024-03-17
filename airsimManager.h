#include <iostream>
#include <memory>

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/StrictMode.hpp"
#include "common/common_utils/FileSystem.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

using namespace msr::airlib;

#include "signalSlots.h"
#include "messageStruct.h"

class AirSimManager {
public:
	static std::shared_ptr<AirSimManager> getInstance() {
		if (instance == nullptr) {
			instance = std::make_shared<AirSimManager>();
		}
		return instance;
	}
	AirSimManager() {}
	~AirSimManager() {}

	SignalSlot::Signal<void(const sensor_msgs::ImageConstPtr& msg)> notifyImage;
	SignalSlot::Signal<void(const sensor_msgs::Imu::ConstPtr& msg_in)> notifyImu;
	SignalSlot::Signal<void(const sensor_msgs::PointCloud2::ConstPtr& msg_in)> notifyPoints;

	void run();

	void getImuData();
	void getImageData(const std::string& camera_name = "front_center", const std::string& vehicle_name = "UAV201");
	void getLidarData(const std::string& lidar_name = "MyLidar1", const std::string& vehicle_name = "UAV201");
private:
	static std::shared_ptr<AirSimManager> instance;

	msr::airlib::MultirotorRpcLibClient client;
};