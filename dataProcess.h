#pragma

#include "messageStruct.h"

class DataProcess {
public:
    DataProcess() {}
    ~DataProcess() {}

    void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg);

private:

};
