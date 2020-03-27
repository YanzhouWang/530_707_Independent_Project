# 530 707 Independent Project
Final project space for JHU RSP

Project report can be found [here](https://www.dropbox.com/s/p3oeg2vh7b18fnu/Final_Project.html?dl=0).

# Package Description and Usage (continuously updated)
Please note that all units are in ROS default [base unit](https://www.ros.org/reps/rep-0103.html).

## needle_description
Contains the URDF of a needle model. Adjustable parameters include: needle length, needle diamter, approximated maximum curvature, and number of needle segments in the mesh.
The reason why the maximum curvature is adjustable is because this curvature is used to generate joint limits for each needle segment, but it does that at the cost of fidality of the linear approximation of the curved needle, because ideally the approximated line segment depends on the curvature itself, which is variable in real world. Adjust the approximated value to suit your need.