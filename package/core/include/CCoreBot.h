#ifndef CCOREBOT_H_
#define CCOREBOT_H_

#define LIDARSIZE 5

class CCoreBot
{
  public:
    CCoreBot();
    ~CCoreBot();
    void ComputeCurrentPose();
    void StateChange();
    void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);

    

  private:
    ros::NodeHandle CoreNODE;
    ros::NodeHandle nh_priv_;

    ros::Subscriber DriveNode;
    ros::Publisher botState;

    double curLinearVel;
    double curAngularVel;

    double tb3Pose;
    double prevtb3Pose;

    

};


#endif