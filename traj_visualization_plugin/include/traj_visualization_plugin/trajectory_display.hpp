#ifndef TRAJECTORY_DISPLAY_H_
#define TRAJECTORY_DISPLAY_H_

#include <tf/transform_listener.h>

#include <rviz/message_filter_display.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/status_property.h>
#include <rviz/ogre_helpers/billboard_line.h>

#include <display_trajectory_msgs/DisplayTrajectoryStamped.h>



#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace trajectory_display_ns
{
  //will later change it to moveit message type
  class trajectory_display: public rviz::MessageFilterDisplay<display_trajectory_msgs::DisplayTrajectoryStamped>{
    Q_OBJECT
  public:
    trajectory_display();
    virtual ~trajectory_display();

  protected:
    virtual void onInitialize();
    virtual void reset();

  private Q_SLOTS:
    void updateLineColorandAlpha();
    void updateLinkChoice();
    void updateRobotDescription();
    void updateLineWidth();

  private:
    void processMessage(const display_trajectory_msgs::DisplayTrajectoryStamped::ConstPtr& msg);
    
    rviz::StringProperty* target_property_, *description_property_;
    rviz::ColorProperty* color_property_;
    rviz::FloatProperty* alpha_property_, *width_property_;
    rviz::StatusProperty* plugin_status_;
    rviz::BillboardLine* path_line_;

    KDL::Tree tree_;
    KDL::Chain chain_r2f_, chain_r2t_; //root to fixed frame, and root to target frame
    KDL::ChainFkSolverPos_recursive* kdlfk_r2t_;
  };
}

#endif
