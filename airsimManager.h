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

#include <mutex>

class AirSimManager {
public:
	static std::shared_ptr<AirSimManager> getInstance() {
		if (m_pInstance == nullptr) {
			m_pInstance = std::make_shared<AirSimManager>();
		}
		return m_pInstance;
	}
	AirSimManager() {}
	~AirSimManager() {}

	SignalSlot::Signal<void(const sensor_msgs::ImageConstPtr& msg)> notifyImage;
	SignalSlot::Signal<void(const sensor_msgs::Imu::ConstPtr& msg_in)> notifyImu;
	SignalSlot::Signal<void(const sensor_msgs::PointCloud2::ConstPtr& msg_in)> notifyPoints;

	void run();
	void getPosition();
	void test();

	void getImuData();
	void getImageData(const std::string& camera_name = "front_center", const std::string& vehicle_name = "UAV201");
	void getLidarData(const std::string& lidar_name = "MyLidar1", const std::string& vehicle_name = "UAV201");
	
	void writeToFile();

protected:
	static std::shared_ptr<AirSimManager> m_pInstance;
	std::mutex m_writeMutex;
	std::queue<std::pair<uint32_t, char*>> m_qBuffers;
	std::string m_sOutFile = "./simdata.bin";


	msr::airlib::MultirotorRpcLibClient client;	
	int m_nFrames = 0;
};