#include "ladybug_camera/decompression/LadybugSeparatorNode.h"

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>


namespace ladybug_camera {

class LadybugSeparatorNodelet : public nodelet::Nodelet {
  boost::shared_ptr<LadybugSeparatorNode> separator_;

  void onInit() override {
    separator_.reset(
        new LadybugSeparatorNode(getNodeHandle(), getPrivateNodeHandle()));
  }
};

PLUGINLIB_EXPORT_CLASS(ladybug_camera::LadybugSeparatorNodelet,
                       nodelet::Nodelet)

} // namespace ladybug_camera
