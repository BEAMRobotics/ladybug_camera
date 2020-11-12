#include "ladybug_camera/decompression/LadybugCombinerNode.h"

namespace ladybug_camera {

LadybugCombinerNode::LadybugCombinerNode(ros::NodeHandle& nh,
                                         ros::NodeHandle& nh_priv)
    : nh_(nh), nh_priv_(nh_priv), img_trpt_(nh) {

  // Get intrinsics for each camera
  for(unsigned int i = 0; i < NUM_CAMS; ++i) {

    const auto istr = std::to_string(i);
    const std::string ns = "/ladybug/camera" + istr;

    nh.getParam("/ladybug/intrinsics/camera" + istr, camera_info_url_);
    std::cout << camera_info_url_ << std::endl;
    camera_info_url_ = "file://" + camera_info_url_;

    ros::NodeHandle cimnh(ns);
    cam_infos_[i].reset(new camera_info_manager::CameraInfoManager(cimnh));
    cam_infos_[i]->setCameraName(std::string("ladybug") + istr);

    if(!cam_infos_[i]->loadCameraInfo(camera_info_url_))
      cam_infos_[i].reset();

  }

  CreatePublishers();
}

void LadybugCombinerNode::CreatePublishers() {

  // Monitor whether anyone is subscribed to the output
  using ConnectCb = image_transport::SubscriberStatusCallback;
  ConnectCb connect_cb = boost::bind(&LadybugCombinerNode::ConnectCb, this);

  // Make sure we don't enter ConnectCb() between advertising and assigning to
  // pub_XXX
  boost::lock_guard<boost::recursive_mutex> lock(mutex_);

  for(unsigned i = 0; i < NUM_CAMS; ++i) {
    const auto istr = std::to_string(i);
    const std::string ns = "/ladybug/camera" + istr;
    const std::string topic = ns + "/image_raw";
    cam_pubs_[i] = img_trpt_.advertiseCamera(topic, 1, connect_cb, connect_cb);
  }
}

// Handles (un)subscribing when clients (un)subscribe
void LadybugCombinerNode::ConnectCb() {

  boost::lock_guard<boost::recursive_mutex> lock(mutex_);
  unsigned int num_subs = 0;
  for(unsigned int i = 0; i < NUM_CAMS; ++i) {
    const unsigned int n = cam_pubs_[i].getNumSubscribers();
    recombiner_.cam_enabled_[i] = (n != 0);
    num_subs += n;
  }
  if(num_subs == 0) {
    sub_raw_.shutdown();
    is_sub_raw_connected_ = false;
  } else if(!is_sub_raw_connected_) {
    sub_raw_ = nh_.subscribe(
        "/ladybug/images", 1, &LadybugCombinerNode::ImageCb, this);
    is_sub_raw_connected_ = true;
  }
}

void LadybugCombinerNode::ImageCb(
    const LadybugTilesCompressedConstPtr& raw_msg) {

  boost::lock_guard<boost::recursive_mutex> lock(mutex_);

  std::vector<sensor_msgs::ImagePtr> images;
  recombiner_.Recombine(*raw_msg, images);

  // publish
  for(unsigned int i = 0; i < NUM_CAMS; ++i) {
    if(recombiner_.cam_enabled_[i] && images[i]) {
      // if we failed to load the configuration file, try to at least provide
      // the size info that we can get from the image.
      if(!cam_infos_[i]) {
        const std::string istr = std::to_string(i);
        const std::string ns = "/ladybug/camera" + istr;

        ros::NodeHandle cim_nh(ns);
        cam_infos_[i].reset(new camera_info_manager::CameraInfoManager(cim_nh));
        cam_infos_[i]->setCameraName(std::string("ladybug") + istr);

        sensor_msgs::CameraInfo cam_info;
        cam_info.header = images[i]->header;
        cam_info.width = images[i]->width;
        cam_info.height = images[i]->height;
        cam_infos_[i]->setCameraInfo(cam_info);
      }

      auto ci = boost::make_shared<sensor_msgs::CameraInfo>(
          cam_infos_[i]->getCameraInfo());
      ci->header = images[i]->header;
      cam_pubs_[i].publish(images[i], ci);

    }
  }
}


} // namespace ladybug_camera
