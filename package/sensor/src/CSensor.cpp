
#include "sensor/include/CSensor.h"

CSensor::CSensor(){

}

CSensor::~CSensor(){
    
}

void CSensor::SensorMsgCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
{
  uint16_t scan_angle[6] = {0, 30,60,90,120,180};

  for (int num = 0; num < 3; num++)
  {
    if (std::isinf(msg->ranges.at(scan_angle[num])))
    {
      scan_data_[num] = msg->range_max;
    }
    else
    {
      scan_data_[num] = msg->ranges.at(scan_angle[num]);
    }
  }
}