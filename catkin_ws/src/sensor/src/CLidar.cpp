#include "CLidar.h"

//---
CLidar::CLidar():nh_priv_("~")
{
  ROS_INFO("Lidar Node initalised");
    // Initialise subscriber
  laserScanSub = nh_.subscribe("scan", QSize, &CLidar::LidarScanMsgCallBack, this);

  //ROS publisher to publish to a new topic
  lidarPub= nh_.advertise<std_msgs::Float64MultiArray>(topicName,QSize);

  // Populate Vector with default 0.0 lidar scan values
  for (int i = 0; i < LidarDataSize; i++)
  {
    ScanData.push_back(0.0);
  }

  // Populate publishing message Float64MultiArray
  // https://answers.ros.org/question/226726/push-vector-into-multiarray-message-and-publish-it/
  // set up dimensions
  msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg.layout.dim[0].size = ScanData.size();
  msg.layout.dim[0].stride = 1;
  msg.layout.dim[0].label = "x"; // or whatever name you typically use to index vec1

  ROS_ASSERT(true);
}

//---
CLidar::~CLidar()
{
  ScanData.clear();
  ScanData.empty();
  ros::shutdown();
}

//---
void CLidar::LidarScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{ 
  // Read in range of lidar measurement at specified angles 
  for (int num = 0; num < LidarDataSize ; num++)
  {
    if (std::isinf(msg->ranges.at(scanAngle[num])))
    {
      ScanData[num] = msg->range_max;
    }
    else
    {
      ScanData[num] = msg->ranges.at(scanAngle[num]);
    }
  }
}

//---
void CLidar::FillPublishData()
{
  // copy in the data
  msg.data.clear();
  msg.data.insert(msg.data.end(), ScanData.begin(), ScanData.end());

  // Publish
  lidarPub.publish(msg);

  // Clear data
  ScanData.clear();
}

//LIDAR NODE
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "Lidar");
  CLidar Lidar;
  ros::Rate loop_rate(125);

  while(ros::ok)
  {
    Lidar.FillPublishData();
    
    // process callback for this node
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}