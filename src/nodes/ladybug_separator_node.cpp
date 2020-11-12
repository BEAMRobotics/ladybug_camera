#include "ladybug_camera/decompression/LadybugSeparatorNode.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "ladybug_separator_node");
  ros::NodeHandle nh, priv_nh("~");
  ladybug_camera::LadybugSeparatorNode separator(nh, priv_nh);
  ros::spin();
  return 0;
}
