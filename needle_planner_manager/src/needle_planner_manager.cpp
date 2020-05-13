#include <ros/ros.h>
#include <moveit/planning_interface/planning_interface.h>
#include <class_loader/class_loader.h>
#include <needle_planner_manager/rrt_needle_planner.hpp>

namespace needle_planning_interface{
  class NeedlePlannerManager: public planning_interface::PlannerManager{
  private:
    planning_interface::PlanningContextPtr context;
    ros::NodeHandle nh;
    
  public:

    NeedlePlannerManager():planning_interface::PlannerManager(){}
   
    ~NeedlePlannerManager(){}

    bool canServiceRequest(const planning_interface::MotionPlanRequest& req)const{
      return true;
    }
    
    std::string getDescription() const{
      return std::string("Needle Planning");
    }
    
    
    void getPlanningAlgorithms(std::vector<std::string>& algs) const{
      algs.resize(1);
      algs[0]=std::string("RRTNeedlePlanner");
    }
    
    planning_interface::PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr &planning_scene,
							      const planning_interface::MotionPlanRequest &req,
							      moveit_msgs::MoveItErrorCodes &error_code) const{
      context->setPlanningScene(planning_scene);
      context->setMotionPlanRequest(req);
      return context;
    }
    
    // this initialize also calls loadPlannerConfigurations method to retrieve planner settings from the parameter server
    bool initialize(const robot_model::RobotModelConstPtr &model, const std::string &ns){
      if(!ns.empty()){
	nh = ros::NodeHandle(ns);
	context.reset(new RRTNeedlePlanner(model,
					 std::string("NeedleRRT"),
					 std::string("UR5Needle"),
					 nh));
      }

      bool pconfig_status = loadPlannerConfigurations("UR5Needle","RRTNeedlePlanner",config_settings_);
      if (!pconfig_status){
	ROS_ERROR("ERROR: Cannot retrieve planner settings.");
	return false;
      }
      return true;
    }

    // get the new parameters pcs from MoveIt, upload it back to the parameter server, then updating local config_settings_
    void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pcs){
      for (auto& x:pcs){
	std::map<std::string, std::string> getconfig = x.second.config;
	for (auto& y:getconfig){
	  nh.setParam("/move_group/planner_configs/RRTNeedlePlanner/"+y.first,y.second);
	}
      }
      bool update_status=loadPlannerConfigurations("UR5Needle","RRTNeedlePlanner",config_settings_);
      if(update_status){
	ROS_INFO("Planner parameters updated.");
      }
    }


    // method to retrieve settings from the parameter server and store it locally to config_settings_
    bool loadPlannerConfigurations(const std::string& group_name,
				       const std::string& planner_name,
				       planning_interface::PlannerConfigurationMap& pconfig){
      pconfig.clear();
      XmlRpc::XmlRpcValue xml_config;
      if(!nh.getParam("/move_group/planner_configs/"+planner_name, xml_config)){
	ROS_ERROR("Could not find planner configurations on the parameter server.");
	return false;
      }
      
      if(xml_config.getType() != XmlRpc::XmlRpcValue::TypeStruct){
	ROS_ERROR("A planning configuration should be of type XmlRpc Struct type (for configuration '%s')",
		  planner_name.c_str());
	return false;
      }


      planning_interface::PlannerConfigurationSettings planner_config;
      planner_config.name=group_name + "[" + planner_name + "]";
      planner_config.group=group_name;
      std::map<std::string, std::string> params;
      for(auto it = xml_config.begin(); it != xml_config.end(); it++){
	if(it->second.getType() == XmlRpc::XmlRpcValue::TypeString){
	  params.insert( {
	    {it->first, static_cast<std::string>(it->second)}
	    });
	}
	else if(it->second.getType() == XmlRpc::XmlRpcValue::TypeDouble){
	  params.insert({
	    {it->first, std::to_string(static_cast<double>(it->second))}
	    });
	}
	else if(it->second.getType() == XmlRpc::XmlRpcValue::TypeInt){
	  params.insert({
	    {it->first, std::to_string(static_cast<int>(it->second))}
	    });
	}
	else if(it->second.getType() == XmlRpc::XmlRpcValue::TypeBoolean){
	  params.insert({
	    {it->first, std::to_string(static_cast<bool>(it->second))}
	    });
	}
      }
      planner_config.config=params;
      pconfig.insert({planner_name, planner_config});

      return true;
    }
  };
}
CLASS_LOADER_REGISTER_CLASS (needle_planning_interface::NeedlePlannerManager, planning_interface::PlannerManager);
