#include <path_generator/path_generator.hpp>

PathGenerator::PathGenerator(ros::NodeHandle& nh): nh_(nh){
  ROS_INFO("In path converter constructor.");
  std::cout<<"Root name: ";
  std::cin>>root_name_;
  std::cout<<"EE name: ";
  std::cin>>ee_name_;
  path_pub_=nh.advertise<nav_msgs::Path>("ee_path",1,true);
}

PathGenerator::~PathGenerator(){
  ROS_INFO("In path converter destructor.");
  delete kdlfk_;
}

bool PathGenerator::getTree(){
  std::string robot_description_string;
  nh_.param("/robot_description",robot_description_string, std::string());
  if(kdl_parser::treeFromString(robot_description_string,tree_)){
    ROS_INFO("Retrieved robot description from parameter server.");
    return true;
  }
  else{
    ROS_ERROR("Cannot get robot descripton.");
    return false;
  }
}

bool PathGenerator::getChain(){
if(tree_.getChain(root_name_,ee_name_,chain_)){
  ROS_INFO_STREAM("Retrieved chain from "<<root_name_<<" to "<<ee_name_);
    return true;
  }
  else{
    ROS_ERROR("Cannot get a chain");
    return false;
  }
}

void PathGenerator::instantiateSolver(){
  kdlfk_=new KDL::ChainFkSolverPos_recursive(chain_);
}

void PathGenerator::startSub(){
  traj_sub_=nh_.subscribe("planned_path",10, &PathGenerator::traj_callback, this);
}

void PathGenerator::traj_callback(const moveit_msgs::DisplayTrajectory& msg){
  int num_msgs=msg.trajectory[0].joint_trajectory.points.size();
  int num_joints=chain_.getNrOfJoints();
  nav_msgs::Path path;
  std::string frame_id = msg.trajectory[0].joint_trajectory.header.frame_id;
  path.poses.resize(num_msgs);
  q_in_.resize(num_joints);
  for(size_t i=0; i<num_msgs; i++){ //for each msg
    for(size_t j=0; j<num_joints; j++){
      q_in_(j)=msg.trajectory[0].joint_trajectory.points[i].positions[j]; //filling joint array
    }
    // use each q_in_ to calculate FK
    kdlfk_->JntToCart(q_in_,p_out_);
    KDL::Vector V=p_out_.p;
    // package path message
    
    path.poses[i].pose.position.x=V[0];
    path.poses[i].pose.position.y=V[1];
    path.poses[i].pose.position.z=V[2];
    
    //    std::cout<<V[0]<<" "<<V[1]<<" "<<V[2]<<std::endl;
  }
  path.header.frame_id=frame_id;
  path_pub_.publish(path);
}
