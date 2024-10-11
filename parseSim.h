#pragma

#include <string>
#include "messageStruct.h"
#include "signalSlots.h"

#include <fstream>

#define MAX_LEN 200000

class ParseSim {
public:
	virtual ~ParseSim() {};

	static ParseSim* getInstance() {
		if (m_pInstance == nullptr) {
			m_pInstance = new ParseSim();
		}
		return m_pInstance;
	}

	SignalSlot::Signal<void(const sensor_msgs::ImageConstPtr& msg)> notifyImage;
	SignalSlot::Signal<void(const sensor_msgs::Imu::ConstPtr& msg_in)> notifyImu;
	SignalSlot::Signal<void(const sensor_msgs::PointCloud2::ConstPtr& msg_in)> notifyPoints;

	void parse(const std::string& sFileName);

protected:
    ParseSim() {}

private:
    static ParseSim* m_pInstance;

	unsigned char clen[4] = { 0 };
	unsigned char buffer[MAX_LEN] = { 0 };
    float m_imuData[6];
	std::vector<uchar> m_vImgdata;
};
