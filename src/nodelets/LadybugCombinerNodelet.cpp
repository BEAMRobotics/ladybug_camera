#include "ladybug_camera/decompression/LadybugCombinerNode.h"

#include <nodelet/nodelet.h>
#include <nodelet_topic_tools/nodelet_throttle.h>
#include <pluginlib/class_list_macros.h>

namespace ladybug_camera {

class LadybugCombinerNodelet : public nodelet::Nodelet {
  boost::shared_ptr<LadybugCombinerNode> recombiner_;
  void onInit() override {
    recombiner_.reset(
        new LadybugCombinerNode(getNodeHandle(), getPrivateNodeHandle()));
  }
};

PLUGINLIB_EXPORT_CLASS(ladybug_camera::LadybugCombinerNodelet,
                       nodelet::Nodelet)

using NodeletThrottleImage = nodelet_topic_tools::NodeletThrottle<sensor_msgs::Image>;
PLUGINLIB_EXPORT_CLASS(ladybug_camera::NodeletThrottleImage,
                       nodelet::Nodelet)

} // namespace ladybug_camera
