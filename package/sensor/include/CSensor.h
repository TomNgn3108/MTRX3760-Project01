#ifndef CSENSOR_H_
#define CSENSOR_H_

#define LIDARSIZE 6

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

class CSensor
{
  public:
    CSensor();
    ~CSensor();
    void SensorMsgCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

  private:
    ros::NodeHandle sensorNode;
    ros::NodeHandle nh_priv_;

    ros::Subscriber sensorSub;
    ros::Publisher sensorData;

    double scan[LIDARSIZE];
    double scanAngle[LIDARSIZE];

   

};


#endif