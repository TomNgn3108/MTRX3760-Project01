#include "TB3Drive.h"

//---
TB3Drive::TB3Drive(): nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node
  ROS_INFO("Turtlebot3 Drive node initalised");

    // initialize ROS parameter
  std::string cmd_vel_topic_name = nh_.param<std::string>("cmd_vel_topic_name", "");

  // initialize variables
  escapeRange        = 30.0 * DEG2RAD;
  checkForwardDist   = 0.7;
  checkSideDist      = 0.6;

  tb3Pose = 0.0;
  prevTB3pose = 0.0;

  angularVel = 0.0;
  linearVel = 0.0;

  // initialize publishers
  cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 1000);

  // initialize subscribers
  cLidarSub = nh_.subscribe("LIDAR", 1000, &TB3Drive::cLidarMsgCallBack, this);
  cBotSub = nh_.subscribe("BOT", 1000, &TB3Drive::cBotMsgCallBack, this);

  ROS_ASSERT(true);
}

//---
TB3Drive::~TB3Drive()
{
  updatecommandVelocity(0.0, 0.0);
  ros::shutdown();
}

//---
void TB3Drive::cLidarMsgCallBack(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  scan_data_.clear();
  for (int i = 0; i < msg->data.size(); i ++){
    scan_data_.push_back(msg->data[i]);
  }
}

//---
void TB3Drive::cBotMsgCallBack(const std_msgs::Float64::ConstPtr &msg)
{
  tb3Pose = msg->data;
}

//---
void TB3Drive::updatecommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_.publish(cmd_vel);
}

//---
bool TB3Drive::controlLoop()
{

  return true;
}

//---
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "Drive_Node");
  TB3Drive drive;

  ros::Rate loop_rate(125);

  while (ros::ok())
  { 
    bool b  = drive.controlLoop();

    // process callback for this node
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
