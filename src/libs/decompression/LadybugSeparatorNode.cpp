#include "ladybug_camera/decompression/LadybugSeparatorNode.h"
#include "ladybug_camera/decompression/pgr_compressor_header_info.h"

namespace ladybug_camera {

void SeparateImages(LadybugTilesCompressed& images,
                    const unsigned char* packet,
                    const ros::Time& stamp) {

  LadybugCompressorHeaderInfo compressor_info;
  compressor_info.parse(packet);

  images.images.resize(24);

  images.header.stamp = stamp;
  images.header.frame_id = "ladybug";

  for(unsigned int i = 0; i < 6; ++i) { // Loop through each cam
    const std::string frame_id = "ladybug" + std::to_string(i);
    for(int j = 0; j < 4; j++) {        // Loop through each channel of cam
      const unsigned int k = i * 4 + j; // Index of current channel (0-24)
      const LadybugCompressorHeaderInfo::ImageInfo& imageinfo =
          compressor_info.getInfo(k);
      images.images[k].format = "jpeg";
      images.images[k].header.frame_id = frame_id;
      images.images[k].header.stamp = stamp;
      images.images[k].data.assign(imageinfo.pData,
                                   imageinfo.pData + imageinfo.size);
    }
  }
}

LadybugTilesCompressedPtr SeparateImages(const sensor_msgs::Image& packet) {

  auto images = boost::make_shared<LadybugTilesCompressed>();

  SeparateImages(*images, &(packet.data[0]), packet.header.stamp);
  return images;
}


LadybugSeparatorNode::LadybugSeparatorNode(ros::NodeHandle& nh,
                                           ros::NodeHandle& nh_priv)
    : img_trpt_(nh) {

  imgs_pub_ = nh.advertise<LadybugTilesCompressed>("/ladybug/images", 5);

  std::string img_raw_topic = "/ladybug/packet";
  img_sub_ =
      img_trpt_.subscribe(img_raw_topic, 5, &LadybugSeparatorNode::ImageCb, this);

}

void LadybugSeparatorNode::ImageCb(const sensor_msgs::ImageConstPtr& img) {
  imgs_pub_.publish(SeparateImages(*img));
}

} // namespace ladybug_camera
