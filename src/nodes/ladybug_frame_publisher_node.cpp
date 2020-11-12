#include "ladybug_camera/LadybugTilesCompressed.h"
#include <ros/ros.h>

namespace ladybug_camera {

/**
 * @brief Simple class for subscribing to the LadybugTilesCompressed and
 * republishing on individual topics.
 */
class LadybugFramePublisher {
public:
  LadybugFramePublisher();

private:
  ros::NodeHandle nh_;
  ros::Publisher img_pubs_[24];
  ros::Subscriber img_subs_;

  void ImageCb(const LadybugTilesCompressedConstPtr& images);
};

LadybugFramePublisher::LadybugFramePublisher() {

  for(unsigned int cam = 0; cam < 6; ++cam) {

    const std::string cam_topic_ns =
        "/ladybug/camera" + std::to_string(cam) + "/";

    for(unsigned int j = 0; j < 4; ++j) {

      const std::string channel_str =
          "bayer_img" + std::to_string(j) + "/compressed";

      const std::string cam_topic = cam_topic_ns + channel_str;

      img_pubs_[cam * 4 + j] =
          nh_.advertise<sensor_msgs::CompressedImage>(cam_topic, 20);
    }
  }

  img_subs_ = nh_.subscribe(
      "/ladybug/images", 10, &LadybugFramePublisher::ImageCb, this);
}

void LadybugFramePublisher::ImageCb(
    const LadybugTilesCompressedConstPtr& images) {

  for(unsigned int i = 0; i < 24; ++i)
    img_pubs_[i].publish(images->images[i]);
}

} // namespace ladybug_camera

int main(int argc, char** argv) {
  ros::init(argc, argv, "ladybug_frame_publisher");
  ladybug_camera::LadybugFramePublisher pubisher;
  ros::spin();
  return 0;
}
