# 530 707 Independent Project
Final project space for JHU RSP

Project report can be found [here](https://www.dropbox.com/s/p3oeg2vh7b18fnu/Final_Project.html?dl=0).

# Package Description and Usage (continuously updated)
Please note that all units are in ROS default [base unit](https://www.ros.org/reps/rep-0103.html).

## needle_description
Contains the URDF of a needle model. Adjustable parameters include: needle length, needle diamter, approximated maximum curvature, and number of needle segments in the mesh.

The reason why the maximum curvature is adjustable is because this curvature is used to generate joint limits for each needle segment, but it does that at the cost of fidality of the linear approximation of the curved needle, because ideally the approximated line segment depends on the curvature itself, which is variable in real world. Adjust the approximated value to suit your need.

The needle can be attached to any other robot. See an example in package ur5_needle_description.

## ur5_needle_description
Contains the URDF of a plain UR5 with a needle attached at its tool0 link.

## path_generator
A node that requires a planning environment (MoveIt! and a robot) and visualizes the planned path for a specific link of the robot in RViz. The topic needs to be remapped to whichever topic that returns the planned path in message type moveit_msg/DisplayTrajectory. Upon initialization, the node asks for the Root link (usually world, base, or base_link) of the kinematic chain, then the EE link of the kinematic chain, which is the link of interest and does not have to be the actual end effector. In RViz, simply add a Path display and subsribe to the correct topic (/ee_path by default) to visualize the planned path. Every time a planning request is sent and a solution is found, the path should appear on the screen.