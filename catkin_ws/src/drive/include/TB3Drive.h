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

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define CENTER 0
#define LEFT   1
#define RIGHT  2

#define LINEAR_VELOCITY  0.3
#define ANGULAR_VELOCITY 1.5

#define GET_TB3_DIRECTION 0
#define TB3_DRIVE_FORWARD 1
#define TB3_RIGHT_TURN    2
#define TB3_LEFT_TURN     3
#define TB3_PID_LEFT      4
#define TB3_PID_RIGHT     5

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

  // Variables
  double escapeRange;
  double checkForwardDist;
  double checkSideDist;

  double forwardTarget;
  double sideTarget;
  
  double maxTurnVel;

  double maxForwardVel;
  double minForwardVel;

  double turnP;
  double forwardP;

  std::vector<double> scan_data_;

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
