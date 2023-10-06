// Original class was turtlebot_drive taken from simulation example package
// Class TB3Drive is the main class that controls the bot based on
// lidar and odom data published by CLidar and CPose nodes.
// online- Authors: Taehun Lim (Darby). Modified by the team to fit with design

#ifndef TB3DRIVE_H_
#define TB3DRIVE_H_

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>


// Lidar indexing 
const int CENTER = 0;
const int LEFT   = 1;
const int RIGHT  =  2;

// Velocity
const double STOP_FOWARD_V = 0.0;

// Bot states 
const int STRAIGHT= 0;
const int LEFT_TURN = 1;
const int CORNER_TURN   = 2;
const int DEFAULT_STATE = 3;


// TB3DRive interface---------------------------------------------------------
// This class controls the robot based on the lidar readings. The class has the 
// capabilities to transit states of the robot and compute linear and angular
// velocities and publish to cmd_vel to set velocity of the bot. 

class TB3Drive
{
 public:
  TB3Drive();
  ~TB3Drive();
  bool controlLoop();

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // ROS Topic Publishers
  ros::Publisher cmd_vel_pub_;              

  // ROS Topic Subscribers
  ros::Subscriber cLidarSub;
  ros::Subscriber cBotSub;

  double forwardTarget;
  double forwardTargetTurn;
  double sideTarget;
  
  double maxTurnVel;

  double maxForwardVel;
  double minForwardVel;

  double angularVel;
  double linearVel;

  double turnKp;               // Proportional gain for angular velocity
  double forwardKp;            // Proportional gain for linear velocity

  double tb3Pose;             // Current Position form odometry - sent to by CPose 
  double prevTB3pose;         // Previous Position from odometry

  int leftTurnFlag;

  std::vector<double>lidarData;

  // Function publishes to cmd_vel topic to control linear
  // and angular velocity of turtlebot.
  void updatecommandVelocity(double linear, double angular);

  // Callback functions receiving messages from CPose class and CLidar class
  void cLidarMsgCallBack(const std_msgs::Float64MultiArray::ConstPtr &msg);
  void cPoseMsgCallBack(const std_msgs::Float64::ConstPtr &msg);
};
#endif 
