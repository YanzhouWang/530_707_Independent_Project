#include <display_trajectory_msg_converter/msg_converter.hpp>

msg_converter::msg_converter(ros::NodeHandle* nodehandle): nh_(*nodehandle){
  traj_pub_=nh_.advertise<display_trajectory_msgs::DisplayTrajectoryStamped>("trajectory_stamped",1 ,true);
  traj_sub_=nh_.subscribe("planned_path",100,&msg_converter::msg_callback, this);
}

msg_converter::~msg_converter(){}

void msg_converter::msg_callback(const moveit_msgs::DisplayTrajectory& msg){
  ROS_INFO("new message received");
  display_trajectory_msgs::DisplayTrajectoryStamped new_msg;
  new_msg.header.stamp=ros::Time::now();
  new_msg.header.frame_id = msg.trajectory[0].joint_trajectory.header.frame_id;
  new_msg.moveit_display_trajectory=msg;
  traj_pub_.publish(new_msg);
}
