#pragma once

#include "ladybug_camera/decompression/LadybugCombiner.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <boost/thread.hpp>

namespace ladybug_camera {

/**
 * @brief Node base for subscribing to the topics with the
 * separated/compressed/rotated 24 images from the ladybug and publishing 6
 * colored images
 */
class LadybugCombinerNode {
public:
  LadybugCombinerNode(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

private:
  /**
   * @brief Create image transport publishers
   */
  void CreatePublishers();

  /**
   * @brief Callback for when someone subscribes to images
   */
  void ConnectCb();

  /**
   * @brief Callback for subscribing to raw images from separator
   * @param raw_msg
   */
  void ImageCb(const LadybugTilesCompressedConstPtr& raw_msg);


  LadybugCombiner recombiner_;

  static constexpr unsigned int NUM_CAMS = 6;
  ros::NodeHandle nh_, nh_priv_;
  ros::Subscriber sub_raw_;
  bool is_sub_raw_connected_ = false;

  std::string camera_info_url_;
  image_transport::ImageTransport img_trpt_;
  image_transport::CameraPublisher cam_pubs_[NUM_CAMS];

  boost::shared_ptr<camera_info_manager::CameraInfoManager>
      cam_infos_[NUM_CAMS];

  boost::recursive_mutex mutex_;

};

} // namespace ladybug_camera
