#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <boost/thread.hpp>
#include <chrono>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>


namespace ladybug_camera {


/**
 * @brief Node base for subscribing to the topics with the
 * separated/compressed/rotated 24 images from the ladybug and publishing 6
 * colored images
 */
class LadybugRectifierNode {
public:
  LadybugRectifierNode(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

  ~LadybugRectifierNode(){
    delete[] weights;
    delete[] r_pixels;
  }
private:
  void CreatePublishers();
  void ConnectCb();
  void ImageCb(const sensor_msgs::ImageConstPtr& raw_msg);
  void GenerateWeights();



  static constexpr unsigned int NUM_CAMS = 6;
  ros::NodeHandle nh_, nh_priv_;
  ros::Subscriber sub_imgs_;

  image_transport::ImageTransport img_trpt_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraPublisher cam_pubs_[NUM_CAMS];

  boost::shared_ptr<camera_info_manager::CameraInfoManager>
      cam_infos_[NUM_CAMS];

  boost::recursive_mutex mutex_;

  cv_bridge::CvImagePtr rect_ptr = boost::make_shared<cv_bridge::CvImage>();
  cv::Mat rect_img{2464, 2048, CV_8UC3, cv::Scalar(0,0,0)};

  std::pair<int, int>* r_pixels = new std::pair<int, int>[2464*2048+100];
  std::tuple<float, float, float, float>* weights = new std::tuple<float, float, float, float>[2464*2048];

  std::string camera_info_url_;
  std::string img_sub_topic_;
  int cam_id_ = 0;
};

} // namespace ladybug_camera
