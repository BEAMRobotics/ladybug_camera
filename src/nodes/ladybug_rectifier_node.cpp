#include "ladybug_camera/decompression/LadybugRectifierNode.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "ladybug_rectify_node");
  ros::NodeHandle nh, priv_nh("~");
  ladybug_camera::LadybugRectifierNode rectify_node(nh, priv_nh);
  ros::spin();
  return 0;
}
