#include <ros/ros.h>
#include <nodelet/loader.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "ladybug_driver_node");

  // This is code based nodelet loading, the preferred nodelet launching is done
  // through roslaunch
  nodelet::Loader nodelet;
  const nodelet::M_string& remap = ros::names::getRemappings();
  nodelet::V_string nargv;
  const std::string& nodelet_name = ros::this_node::getName();
  nodelet.load(
      nodelet_name, "ladybug_camera/LadybugDriverNodelet", remap, nargv);

  ros::spin();

  return 0;
}
