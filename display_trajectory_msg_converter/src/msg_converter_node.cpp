#include <display_trajectory_msg_converter/msg_converter.hpp>


int main(int argc, char** argv){
  ros::init(argc, argv, "msg_converter_node");
  ros::NodeHandle nh;
  msg_converter converter(&nh);
  ros::spin();
  return 0;
}
