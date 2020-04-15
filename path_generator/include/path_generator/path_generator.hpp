#ifndef PATH_GENERATOR_H_
#define PATH_GENERATOR_H_

#include <memory>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

class PathGenerator{
private:
  KDL::ChainFkSolverPos_recursive* kdlfk_;
  
  ros::NodeHandle nh_;
  ros::Subscriber traj_sub_;
  ros::Publisher path_pub_;
  
  KDL::JntArray q_in_;
  KDL::Frame p_out_;
  KDL::Tree tree_;
  KDL::Chain chain_;

  std::string root_name_;
  std::string ee_name_;
public:
  PathGenerator(ros::NodeHandle& nh);
  ~PathGenerator();
  bool getTree();
  bool getChain();
  void instantiateSolver();
  void startSub();
  void startPub();
  void traj_callback(const moveit_msgs::DisplayTrajectory& msg);
};
#endif
