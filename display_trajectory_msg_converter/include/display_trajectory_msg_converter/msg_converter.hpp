#ifndef MSG_CONVERTER_H_
#define MSG_CONVERTER_H_

#include <ros/ros.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <display_trajectory_msgs/DisplayTrajectoryStamped.h>

class msg_converter{
private:
  ros::NodeHandle nh_;
  ros::Subscriber traj_sub_;
  ros::Publisher traj_pub_;

public:
  msg_converter(ros::NodeHandle* nodehandle);
  ~msg_converter();
  void msg_callback(const moveit_msgs::DisplayTrajectory& msg);
};

#endif
