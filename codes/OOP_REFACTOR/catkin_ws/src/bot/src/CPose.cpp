#include "CPose.h"

// Implementation file for class CLidar
// Functions :
//            - Constructor
//            - Destructor
//            - Call back function sub to odom msg
//            - Pose Publshing function
//            - Trajectory plotting function

//---Constructor
CPose::CPose():nh_priv_("~")
{ 
  ROS_INFO("Pose node initalised");
  // Subscribe to odometry topic
  odomSub = nh_.subscribe("odom", QSize, &CPose::odomMsgCallBack, this);

  //ROS publisher to publish to a new topic
  botPub = nh_.advertise<std_msgs::Float64>(topicName,QSize);
  TrajectoryPub = nh_.advertise<visualization_msgs::Marker>(trajectoryTopic,QSize);
  
  
  // Pose data from odometry
  tb3Pose= 0.0;


  ROS_ASSERT(true);
}

//--- Destructor
CPose::~CPose()
{ 
  trajectoryMsg.points.clear();
  ros::shutdown;
}

//--- Call back function sub to odom msg
void CPose::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  // Compute current odometry
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

	tb3Pose = atan2(siny, cosy);
  odomPose = msg->pose.pose;
}

//---Publshing function
void CPose::PublishPose()
{
  msg.data = tb3Pose;
  botPub.publish(msg);
}

//--- Function publishes visualisation markers for path plot
void CPose::TrajectoryVisualise()
{ 
  // TRAJECTORY MSGS 
  //http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes
  // Modify trajectory msg for publhsing
  trajectoryMsg.header.frame_id = "map";            // which frame to use
  trajectoryMsg.header.stamp = ros::Time();         // timing
  trajectoryMsg.frame_locked = true;                // Lock frame 

  // set id and namespace for rViz to access points to plot
  trajectoryMsg.ns = "points";                      
  trajectoryMsg.id = 0;

  // Set shape type of points and  tell msgs to add points
  // on rviz
  trajectoryMsg.type = visualization_msgs::Marker::POINTS;
  trajectoryMsg.action = visualization_msgs::Marker::ADD;

  // Set initial position of trajectory on the map
  trajectoryMsg.pose.position.x = 0.0;
  trajectoryMsg.pose.position.y = 0.0;
  trajectoryMsg.pose.position.z = 0.0;

  // Set initialorienation of trajectory on the map
  trajectoryMsg.pose.orientation.x = 0.0;
  trajectoryMsg.pose.orientation.y = 0.0;
  trajectoryMsg.pose.orientation.z = 0.0;
  trajectoryMsg.pose.orientation.w = 1.0;

  // Size of plot points
  trajectoryMsg.scale.x = 0.05;
  trajectoryMsg.scale.y = 0.05;
  trajectoryMsg.scale.z = 0.05;

  // Set color of plot  - BLUE
  trajectoryMsg.color.r = 0.0f;
  trajectoryMsg.color.g = 0.0f;
  trajectoryMsg.color.b = 1.0f;
  trajectoryMsg.color.a = 1.0;

  // Mesh resoruce
  trajectoryMsg.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

  // Let points plotted exist on the map as long as simulation is running
  trajectoryMsg.lifetime = ros::Duration();

  // store list of odom pose.position
  trajectoryMsg.points.push_back(odomPose.position);
  TrajectoryPub.publish(trajectoryMsg);
}
//-------------------------------------------------------------
// CPose NODE 
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "Pose_Node");
  CPose bot;
  ros::Rate loop_rate(500);

  while(ros::ok)
  { 
    bot.PublishPose();
    bot.TrajectoryVisualise();
    // process callback for this node
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}