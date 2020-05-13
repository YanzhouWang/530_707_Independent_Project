#include <needle_planner_manager/rrt_needle_planner.hpp>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/planning_scene/planning_scene.h>


RRTNeedlePlanner::RRTNeedlePlanner(const robot_model::RobotModelConstPtr& model,
				   const std::string& name,
				   const std::string& group,
				   const ros::NodeHandle& nh):
  planning_interface::PlanningContext(name,group),
  robot_model_(model){}

RRTNeedlePlanner::~RRTNeedlePlanner(){}

bool RRTNeedlePlanner::solve(planning_interface::MotionPlanResponse& res){
  res.trajectory_.reset(new robot_trajectory::RobotTrajectory(robot_model_,
							      getGroupName()));
  res.trajectory_->clear();
  // copy the initial/final joints configurations to vectors qfin and qini
  // This is mainly for convenience.
  std::vector<double> qstart, qfinal;
  for ( size_t i = 0; i < robot_model_->getVariableCount(); i++ ) {
    qfinal.push_back(request_.goal_constraints[0].joint_constraints[i].position);
    qstart.push_back(request_.start_state.joint_state.position[i]);
  }

  ros::Time begin=ros::Time::now();

  //TODO solve


  ros::Time end=ros::Time::now();
  // set the planning time
  ros::Duration duration = end - begin;
  res.planning_time_ = duration.toSec();
  res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

  return true;
}

bool RRTNeedlePlanner::solve(planning_interface::MotionPlanDetailedResponse& res){
  return true;
}

void RRTNeedlePlanner::clear(){}
bool RRTNeedlePlanner::terminate(){return true;}
