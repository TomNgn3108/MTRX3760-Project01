#include "TB3Drive.h"

//---
TB3Drive::TB3Drive(): nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node
  ROS_INFO("Turtlebot3 Drive node initalised");

    // initialize ROS parameter
  std::string cmd_vel_topic_name = nh_.param<std::string>("cmd_vel_topic_name", "");

  // initialize variables
   //escapeRange        = 30.0 * DEG2RAD;
  // checkForwardDist   = 0.7;
  // checkSideDist      = 0.6;

  tb3Pose = 0.0;
  prevTB3pose = 0.0;

  angularVel = 0.0;
  linearVel = 0.0;

  // Populate Vector with default 0.0 lidar scan values
  for (int i = 0; i < 3; i++)
  {
    lidarData.push_back(0.0);
  }

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
  lidarData.clear();
  for (int i = 0; i < msg->data.size(); i ++){
    lidarData.push_back(msg->data[i]);
  }

  ROS_INFO("lefT: %f | MID: %f | Right: %f ",lidarData[0],lidarData[1],lidarData[2]);
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
  // Get range from 3 angles            // Angles (Degrees)
  double xMid = lidarData[CENTER];      // 0
  double xRight = lidarData[RIGHT];     // 45
  double xLeft = lidarData[LEFT];       // 315
  double yLeft = lidarData[3];          // 90
  double yRight = lidarData[4];         // 270

  // Compute distance from measurements of left and right side to middle
  double leftDiff = sqrt(pow(xMid,2) + pow(xLeft,2) - 2 * xMid * xLeft * cos(ANGLE));
  double RightDiff = sqrt(pow(xMid,2) + pow(xRight,2) - 2 * xMid * xRight * cos(ANGLE));

  // Compute x distance from left,right range to mid range
  double leftVert = xLeft * cos(ANGLE);
  double leftHoz = xLeft * sin(ANGLE);

  double rightVert = xRight * cos(ANGLE);
  double rightHoz = xRight * sin(ANGLE);
  

  // Get left and right angles between Diff and Hoz distances in radians
  double leftAngle = acos(leftHoz/leftDiff);
  double rightAngle = acos(rightHoz/RightDiff);
  //double deltaAng = fabs(leftAngle - rightAngle);
  double deltaAng = leftAngle - rightAngle;
  // Main decision control
  // STRAIGHT MOTION


  linearVel = xMid * LINEAR_VELOCITY * FORWARD_P;
  angularVel = deltaAng * ANGULAR_P * ANGULAR_VELOCITY;


  if ( yLeft >= 3.49 && yRight >= 3.49 && xLeft >= 3.49 && xRight >= 3.49 ){
    linearVel = 0.0;
    angularVel = 0.0;
  }
  updatecommandVelocity(linearVel,angularVel);

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
