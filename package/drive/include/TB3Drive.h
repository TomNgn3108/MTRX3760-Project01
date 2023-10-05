/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#ifndef TB3DRIVE_H_
#define TB3DRIVE_H_

#include  <ros/ros.h>

#include  <sensor_msgs/LaserScan.h>
#include  <geometry_msgs/Twist.h>
#include  <nav_msgs/Odometry.h>
#include  <std_msgs/Float64.h>
#include  <std_msgs/Float64MultiArray.h>
#include  <vector>
#include  <cmath>

// Angle conversions macros
#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

// Indexing constants of lidar data
const int CENTER = 0;
const int LEFT  = 2;
const int RIGHT =  1;

// Constant angle differenes between each measurement
const double ANGLE = 45.0;

// Controller constants P-controller
const double FORWARD_P = 0.2;
const double ANGULAR_P = 0.5;

const double LINEAR_VELOCITY  = 0.4;
const double ANGULAR_VELOCITY = 3.0;


class TB3Drive
{
 public:
  TB3Drive();
  ~TB3Drive();
  bool controlLoop();
  void ComputeMotion();

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // ROS Topic Publishers
  ros::Publisher cmd_vel_pub_;

  // ROS Topic Subscribers
  ros::Subscriber cLidarSub;
  ros::Subscriber cBotSub;

  // Variables
  // double escapeRange;
  // double checkForwardDist;
  // double checkSideDist;

  // double forwardTarget;
  // double sideTarget;
  
  double maxTurnVel;

  double maxForwardVel;
  double minForwardVel;

  double turnP;
  double forwardP;

  std::vector<double>lidarData;

  double angularVel;
  double linearVel;

  double tb3Pose;
  double prevTB3pose;

  // Function prototypes
  void updatecommandVelocity(double linear, double angular);

  // Callback functions
  void cLidarMsgCallBack(const std_msgs::Float64MultiArray::ConstPtr &msg);
  void cBotMsgCallBack(const std_msgs::Float64::ConstPtr &msg);
};
#endif 
