#ifndef CLIDAR_H_
#define CLIDAR_H_

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <std_msgs/Float64MultiArray.h>

const char topicName[] = "LIDAR";
const int LidarDataSize = 9;
const int scanAngle[] = {0, 30, 60,90,120,150,180,210,240};

// Set queue size big to prevent loss if any 
// delay occurs
//https://stackoverflow.com/questions/56444248/reason-to-set-queue-size-of-ros-publisher-or-subscriver-to-a-large-value
const int QSize = 1000;      
                                

/// @brief---
// CLidar interface------------------------------------------------------
// This class is for storing the lidar data and publish to its own topic
// for drive class to listen to and get the data. It serves as a class
// that subscribes to the internal lidar scan topic and store it in a 
// vector. 
class CLidar{
  public:
    CLidar();
    ~CLidar();
    void LidarScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
    void FillPublishData();

	private:
		// ROS NodeHandle
  	ros::NodeHandle nh_;
  	ros::NodeHandle nh_priv_;

    // ROS Subscriber to listen in lidar
    ros::Subscriber laserScanSub;

    // ROS publisher to publish to a new topic
    ros::Publisher lidarPub;
    
		// Data for publishing
    std::vector<double> ScanData;
    std_msgs::Float64MultiArray msg;
};




#endif