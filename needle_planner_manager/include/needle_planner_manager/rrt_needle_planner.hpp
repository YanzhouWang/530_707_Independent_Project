#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <needle_planner_manager/needle_state_solver.hpp>

//forward declaration of moveit classes
MOVEIT_CLASS_FORWARD(RRTNeedlePlanner);

class RRTNeedlePlanner: public planning_interface::PlanningContext{

typedef std::vector<double> vertex;
typedef std::pair<vertex, vertex> edge;
typedef std::vector<size_t> path;

private:
robot_model::RobotModelConstPtr robot_model_;
NeedleStateSolver* solver_;
public:
  RRTNeedlePlanner(const robot_model::RobotModelConstPtr& model,
		   const std::string& name,
		   const std::string& group,
		   const ros::NodeHandle& nh=ros::NodeHandle("~"));
  virtual ~RRTNeedlePlanner();
  virtual  void clear();
  virtual  bool solve(planning_interface::MotionPlanResponse& res);
  virtual  bool solve(planning_interface::MotionPlanDetailedResponse& res);
  virtual  bool terminate();
  
  // nonholonomic rrt planning for needle steering
  //virtual bool isStateCollide(const vertex& configuration)=0;
  //virtual vertex generateRandomValid(const vertex& goal)=0;
  //virtual vertex findNearestNeighbor(const std::vector<edge>& E, const std::vector<vertex>& V)=0;
  //virtual std::vector<vertex> generateBranchFromInput(const vertex& root, const std::vector<double>& inputs)=0;
  //virtual path findPath()=0;
};
