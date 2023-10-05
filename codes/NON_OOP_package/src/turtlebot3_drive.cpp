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

#include "turtlebot3_gazebo/turtlebot3_drive.h"
//#include "turtlebot3_gazebo/ccontrol.h"

Turtlebot3Drive::Turtlebot3Drive()
  : nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node
  ROS_INFO("TurtleBot3 Simulation Node Init");
  auto ret = init();
  ROS_ASSERT(ret);
}

Turtlebot3Drive::~Turtlebot3Drive()
{
  updatecommandVelocity(0.0, 0.0);
  ros::shutdown();
}

/*******************************************************************************
* Init function
*******************************************************************************/

bool Turtlebot3Drive::init()
{
  // initialize ROS parameter
  std::string cmd_vel_topic_name = nh_.param<std::string>("cmd_vel_topic_name", "");

  // initialize variables
  escape_range_       = 30.0 * DEG2RAD;
  check_forward_dist_ = 0.3;
  check_side_dist_    = 0.5;

  tb3_pose_ = 0.0;
  prev_tb3_pose_ = 0.0;

  // ------------------------------------------------------
  forward_target_     = 0.3;
  side_target_        = 0.25;  

  forward_target_turn_= 0.3;

  max_turn_velocity_  = 1;

  max_forward_velocity_ = 0.15;
  min_forward_velocity_ = 0;  

  forward_p_ = 0.5;
  turn_p_ = 6;

  left_turn_flag = 3;

  //--------------------------------------------------------

  // initialize publishers
  //cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);
  cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10); // drive ttb motor

  // initialize subscribers
  laser_scan_sub_  = nh_.subscribe("scan", 10, &Turtlebot3Drive::laserScanMsgCallBack, this);
  odom_sub_ = nh_.subscribe("odom", 10, &Turtlebot3Drive::odomMsgCallBack, this);

  return true;
}

void Turtlebot3Drive::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

	tb3_pose_ = atan2(siny, cosy);
}

void Turtlebot3Drive::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  uint16_t scan_angle[3] = {0, 45, 315};

  for (int num = 0; num < 3; num++)
  {
    if (std::isinf(msg->ranges.at(scan_angle[num])))
    {
      scan_data_[num] = msg->range_max;
      ROS_INFO("[laserCallback]: detected infinity range");
    }
    else
    {
      scan_data_[num] = msg->ranges.at(scan_angle[num]);
    }
    
    if( scan_data_[num]==0)
    {
      scan_data_[num] = msg->range_max;
    }
  }
}

void Turtlebot3Drive::updatecommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;scan_data_[2];

  cmd_vel_pub_.publish(cmd_vel);
}

/*******************************************************************************
* wall fellower
*******************************************************************************/

bool Turtlebot3Drive::controlLoop()
{ 

  // check for left turn flag
  if ((scan_data_[CENTER] <= forward_target_)&&(left_turn_flag==0))
  {
    left_turn_flag = 1;  // check for left turn flag
    ROS_INFO("[left turn flag ]: left turn flag is set");
  }
  else if((scan_data_[CENTER] != 0)&&(left_turn_flag==3))
  {
    left_turn_flag = 0;

  }

  // angular velocity
  ang_v_ = turn_p_*(side_target_-scan_data_[RIGHT]);

  if(ang_v_ > max_turn_velocity_)
  {
    ang_v_ = max_turn_velocity_;
  }
  else if(ang_v_ < (-1)*max_turn_velocity_)
  {
    ang_v_ = (-1)*max_turn_velocity_;
  }

  ROS_INFO("[angular velocity]: side_target is %f, Right scan is %f,  ang_v is %f",side_target_, scan_data_[RIGHT], ang_v_);
  
  // linear velocity
  //lin_v_ = forward_p_*(scan_data_[CENTER] - forward_target_)- fabs(ang_v_);

  //lin_v_ = forward_p_*(scan_data_[CENTER] - forward_target_);

  lin_v_ = max_forward_velocity_;

  if(lin_v_ > max_forward_velocity_)
  {
    lin_v_ = max_forward_velocity_;
  }
  else if(lin_v_ <= min_forward_velocity_)
  {
    lin_v_ = min_forward_velocity_;
  }

  //ROS_INFO("[linear velocity]: forward scan is %f, forwards_target is %f, ang_v is %f, lin_v is %f",scan_data_[CENTER], forward_target_, ang_v_, lin_v_);

  if ( left_turn_flag >= 1)  // if left turn flag set, go left turn, otherwise do normal right wall fellower
  {
    lin_v_ = 0;
    ang_v_ = max_turn_velocity_;

    if((scan_data_[CENTER] >= forward_target_turn_)&&(left_turn_flag == 1))// if left turn 90 degree, go for normal right wall fellower, set flag to 0
    {
      left_turn_flag = 2;
    }
    else if((scan_data_[RIGHT] >= side_target_)&&(left_turn_flag==2)) // if left turn 90 degree, go for normal right wall fellower, set flag to 0
    {
      left_turn_flag = 0;
    }
  }
//  else // right wall fellower
//  {
    //ROS_INFO("[left turn flag ]: left turn flag is erase");
//  }
  ROS_INFO("[left turn flag ]: flag is %d", left_turn_flag);
  //ROS_INFO("[linear velocity]: forward scan is %f, forwards_target is %f, ang_v is %f, lin_v is %f",scan_data_[CENTER], forward_target_, ang_v_, lin_v_);

  ROS_INFO("[velocity]:ang_v is %f, lin_v is %f", ang_v_, lin_v_);
  updatecommandVelocity(lin_v_, ang_v_);

  return true;
}

// --------------------------------------------------------------------------------------

// -----------------------Gavin edit-----------------------------------------
void Turtlebot3Drive::report()
{
  //ROS_INFO("hello world");
  //ROS_INFO("scan data at 0 degree is: %f, at 30 degree is %f, at 150 is %f", scan_data_[0], scan_data_[1], scan_data_[2]);
  ROS_INFO("pose is %f", tb3_pose_);
  //updatecommandVelocity(0.1, 0.0);

}
// ------------------------Gavin edit-----------------------------------------

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "turtlebot3_drive");
  Turtlebot3Drive turtlebot3_drive;

  ros::Rate loop_rate(500);

  while (ros::ok())
  {
    turtlebot3_drive.controlLoop();
    //turtlebot3_drive.report();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
