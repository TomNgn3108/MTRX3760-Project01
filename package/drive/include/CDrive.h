#ifndef CDRIVE_H_
#define CDRIVE_H_

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
// CDrive Interface --------------------------------------------
// This class drives the robot with a closed loop system. It subsrcibes
// to 
class CDrive
{
  public:
    CDrive();
    ~CDrive();

    void UpdateVelocity( double linear, double angular);
    void ComputeNextVelocity();

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    
    ros::Publisher driveBot;
    ros::Subscriber botStatus;

    double escapeRangeTurn;
    double escapeRangeAdjust;

    double checkFwDist;
    double maxFwDist;
    

    double maxLeft;
    double mminLeft;

    // 
    void ComputeNextVelocity();
    

};


#endif