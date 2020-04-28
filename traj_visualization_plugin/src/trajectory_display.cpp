#include <traj_visualization_plugin/trajectory_display.hpp>

namespace trajectory_display_ns{
  trajectory_display::trajectory_display(){
    description_property_=new rviz::StringProperty("Robot Description","robot_description",
						   "The name of the ROS parameter where the URDF for the robot is loaded",
						   this, SLOT (updateRobotDescription()));
    target_property_=new rviz::StringProperty("Target link", " ",
					      "Target link of the robot",
					      this, SLOT (updateLinkChoice()));
    color_property_=new rviz::ColorProperty("Line color",QColor(204,51,204),
					    "Color of the path",
					    this, SLOT (updateLineColorandAlpha()));
    alpha_property_=new rviz::FloatProperty("Line alpha",1,
					    "Line alpha, 1 is opaque, 0 is transparent",
					    this, SLOT (updateLineColorandAlpha()));
    width_property_=new rviz::FloatProperty("Line width",0.003,
					    "Line width",
					    this, SLOT(updateLineWidth()));
    plugin_status_=new rviz::StatusProperty("Plugin status","plugin status",rviz::StatusProperty::Level(0) , this);
    plugin_status_->hide();
  }

  trajectory_display::~trajectory_display(){
    delete target_property_;
    delete description_property_;
    delete color_property_;
    delete alpha_property_;
    delete width_property_;
    delete plugin_status_;
    delete kdlfk_r2t_;
  }

  void trajectory_display::onInitialize(){
    MFDClass::onInitialize();
    std::string robot_description_string;
    ros::param::get(description_property_->getStdString(),robot_description_string);
    if(!kdl_parser::treeFromString(robot_description_string, tree_)){
      ROS_ERROR("Cannot build tree.");
      setStatus(rviz::StatusProperty::Level(2),"Robot state","Cannot get robot model from parameter server.");
      plugin_status_->setLevel(rviz::StatusProperty::Level(2));
    }
    else{
      ROS_INFO("Retrieved tree from parameter server.");
      setStatus(rviz::StatusProperty::Level(0),"Robot state","OK");
      plugin_status_->setLevel(rviz::StatusProperty::Level(0));
    }
    path_line_=new rviz::BillboardLine(scene_manager_, scene_node_);
  }

  void trajectory_display::reset(){
    MFDClass::reset();
    updateLinkChoice();
    path_line_->clear();
  }

  void trajectory_display::updateLinkChoice(){
    std::map<std::string, KDL::TreeElement>::const_iterator root;
    root=tree_.getRootSegment();
    std::string root_link=root->first;
    std::string target_link=target_property_->getStdString();
    
    //Setting up for chain from root to target link
    if(!tree_.getChain(root_link,target_link,chain_r2t_)){
      ROS_ERROR("Cannot get chain from %s to %s.", root_link.c_str(),target_link.c_str());
      setStatus(rviz::StatusProperty::Level(2),"Target link","Cannot get chain.");
      plugin_status_->setLevel(rviz::StatusProperty::Level(2));
    }
    else
      {
	ROS_INFO("To-target chain updated from %s to %s.",root_link.c_str(),target_link.c_str());
	kdlfk_r2t_=new KDL::ChainFkSolverPos_recursive(chain_r2t_);
	setStatus(rviz::StatusProperty::Level(0),"Target link","OK");
	plugin_status_->setLevel(rviz::StatusProperty::Level(0));
      }
  }

  void trajectory_display::updateRobotDescription(){
    onInitialize();
  }
  void trajectory_display::updateLineColorandAlpha(){
    Ogre::ColourValue new_color=color_property_->getOgreColor();
    new_color.a=alpha_property_->getFloat();
    path_line_->setColor(new_color.r, new_color.g, new_color.b, new_color.a);
    context_->queueRender();
  }

  void trajectory_display::updateLineWidth(){
    float new_width=width_property_->getFloat();
    path_line_->setLineWidth(new_width);
    context_->queueRender();
  }

 
  void trajectory_display::processMessage(const display_trajectory_msgs::DisplayTrajectoryStamped::ConstPtr& msg){
    path_line_->clear();

    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    if( !context_->getFrameManager()->getTransform( msg->header.frame_id,
						    msg->header.stamp,
						    position, orientation ))
      {
	ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
		   msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
	return;
      }
    path_line_->setPosition(position);
    path_line_->setOrientation(orientation);

    //Proceed only when status is OK
    if(plugin_status_->getLevel()==0){
      int num_wayPoints=msg->moveit_display_trajectory.trajectory[0].joint_trajectory.points.size();
      int num_joints_to_target=chain_r2t_.getNrOfJoints();

      updateLineWidth();
      updateLineColorandAlpha();
    
      //Here is the tricky part and will need more work into this.
      KDL::JntArray  q_in_target;
      q_in_target.resize(num_joints_to_target);
      for(size_t i=0; i<num_wayPoints; i++){
	for(size_t k=0; k<num_joints_to_target; k++){
	  q_in_target(k)=msg->moveit_display_trajectory.trajectory[0].joint_trajectory.points[i].positions[k];
	}
	KDL::Frame  p_out_target;
	kdlfk_r2t_->JntToCart(q_in_target, p_out_target);
	KDL::Vector V=p_out_target.p;
	Ogre::Vector3 new_point{(double)V.x(), (double)V.y(), (double)V.z()};
	path_line_->addPoint(new_point);
      }
    }
  }
}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(trajectory_display_ns::trajectory_display, rviz::Display)
