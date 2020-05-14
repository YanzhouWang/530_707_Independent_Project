# 530 707 Independent Project
Final project space for JHU RSP

Project report can be found [here](https://www.dropbox.com/s/p3oeg2vh7b18fnu/Final_Project.html?dl=0).


# Package Description and Usage (continuously updated)
Please note that all units are in ROS default [base unit](https://www.ros.org/reps/rep-0103.html).

## display_trajectory_msgs
A Stamped extension of moveit_msgs/DisplayTrajectory. This extension allows RViz plugins to built in order to work on the returned trajectory via ROS topics.

The result is a simple message type display_trajectory_msgs/DisplayTrajectoryStamped.

## display_trajectory_msg_converter
A single node that subscribes to topic under the message type moveit_msgs/DisplayTrajectory, and reworks it into display_trajectory_msgs/DisplayTrajectoryStamped. The default outlet is /trajectory_stamped, which can be used in the traj_visualization_plugin.

It is recommended to start this node via a launch file with the robot, such as demo.launch in any robot-specific MoveIt! packages.

## traj_visualization_plugin
An RViz plugin that allows visualization of a planned trajectory by MoveIt!. The topic should take in messages of type display_trajectory_msgs/DisplayTrajectoryStamped.
The Target link field should be filled with a desired link in the robot kinematic chain.

Note that in order for the plugin to behave correctly, the entire kinematic chain should be set up as a planning group. For example, in case of a UR5 robot with a needle attached, the UR5+needle should be bundled together as one planning group. The reason being that the trajectory returned does not have any information about the planning group; if the UR5 and needle are two planning groups, there is no way to tell (at least from a publisher/subscriber perspective) which planning group this returned trajectory is for. A correct example is set up in the companion package ur5_needle_moveit_config.

## needle_planner_manager
A custom planner manager and a planner prototpye for needle steering using RRT. The planner manager exposes the parameters that are relavent to nonhonolomic planning for needle steering using RRT. The parameters can be changed in ur5_needle_moveit_config/config/needle_planning.yaml.

## needle_description
Contains the URDF of a needle model. Adjustable parameters include: needle length, needle diamter, approximated maximum curvature, and number of needle segments in the mesh.

The reason why the maximum curvature is adjustable is because this curvature is used to generate joint limits for each needle segment, but it does that at the cost of fidality of the linear approximation of the curved needle, because ideally the approximated line segment depends on the curvature itself, which is variable in real world. Adjust the approximated value to suit your need.

The needle can be attached to any other robot. See an example in package ur5_needle_description.

## ur5_needle_description
Contains the URDF of a plain UR5 with a needle attached at its tool0 link.

## ur5_needle_moveit_config
Companion MoveIt! package for ur5_needle_description, traj_visualization_plugin, needle_planner_manager, display_trajectory_msgs and display_trajectory_msgs_converter. The demo.launch file is modified such that display_trajectory_msg_converter node is launched and remapped to the correct topic, and the default planning library is set to use the custom library. The config file is also modified to look for traj_visualization_plugin upon launch.

## path_generator (REMOVED)
A node that requires a planning environment (MoveIt! and a robot) and visualizes the planned path for a specific link of the robot in RViz. The topic needs to be remapped to whichever topic that returns the planned path in message type moveit_msg/DisplayTrajectory. Upon initialization, the node asks for the Root link (usually world, base, or base_link) of the kinematic chain, then the EE link of the kinematic chain, which is the link of interest and does not have to be the actual end effector. In RViz, simply add a Path display and subsribe to the correct topic (/ee_path by default) to visualize the planned path. Every time a planning request is sent and a solution is found, the path should appear on the screen.

This package is replaced by a MoveIt/RViz plugin: traj_visualization_plugin

