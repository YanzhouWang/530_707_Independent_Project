#include <path_generator/path_generator.hpp>

int main(int argc, char** argv){
  ros::init(argc, argv,"path_generator_node");
  ros::NodeHandle nh;
  PathGenerator path_generator(nh);
  if(path_generator.getTree()){
    if(path_generator.getChain()){
      path_generator.instantiateSolver();
      path_generator.startSub();
    }
  }
  ros::spin();
}
