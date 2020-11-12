#pragma once

#include "ladybug_camera/LadybugTilesCompressed.h"
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>


namespace ladybug_camera {

/**
 * @brief Node base for subscribing to ladybug/image_raw topic containing raw
 * image packet, separating using the LadybugSeparator class, and publishing
 * the 24 separated/compressed images as ladybug_msgs/LadybugImagesCompressed
 * msg
 */
class LadybugSeparatorNode {
public:
  LadybugSeparatorNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);

private:

  void ImageCb(const sensor_msgs::ImageConstPtr&);

  ros::Publisher imgs_pub_;
  image_transport::ImageTransport img_trpt_;
  image_transport::Subscriber img_sub_;
};

} // namespace ladybug_camera
