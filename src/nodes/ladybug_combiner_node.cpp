#include "ladybug_camera/decompression/LadybugCombinerNode.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "ladybug_recombine_node");
  ros::NodeHandle nh, priv_nh("~");
  ladybug_camera::LadybugCombinerNode recombine_node(nh, priv_nh);
  ros::spin();
  return 0;
}
