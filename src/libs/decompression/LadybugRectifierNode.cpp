#include "ladybug_camera/decompression/LadybugRectifierNode.h"
#include <cmath>
#include <ladybug/ladybug.h>
#include <ladybug/ladybuggeom.h>
#include <ladybug/ladybugrenderer.h>

namespace ladybug_camera {

LadybugRectifierNode::LadybugRectifierNode(ros::NodeHandle& nh,
                                           ros::NodeHandle& nh_priv)
    : nh_(nh), nh_priv_(nh_priv), img_trpt_(nh){


  nh_priv.getParam("cam_id",cam_id_);


    const auto istr = std::to_string(cam_id_);
    const std::string ns = "/ladybug/camera" + istr + "/rectify";

    nh.getParam("/ladybug/intrinsics/camera" + istr, camera_info_url_);
    camera_info_url_ = "file://" + camera_info_url_;

    ros::NodeHandle cimnh(ns);
    cam_infos_[cam_id_].reset(new camera_info_manager::CameraInfoManager(cimnh));
    cam_infos_[cam_id_]->setCameraName(std::string("ladybug") + istr);

    if(!cam_infos_[cam_id_]->loadCameraInfo(camera_info_url_))
      cam_infos_[cam_id_].reset();

  img_sub_topic_ = "/ladybug/camera"+std::to_string(cam_id_)+"/image_color";

  img_sub_ = img_trpt_.subscribe(img_sub_topic_, 1, &LadybugRectifierNode::ImageCb, this);

  GenerateWeights();
  CreatePublishers();
}

void LadybugRectifierNode::GenerateWeights() {

  std::cout << "Generating weights for cmaera: " << cam_id_ << std::endl;
//  ROS_INFO("Generating weights for camera : ");
  boost::lock_guard<boost::recursive_mutex> lock(mutex_);

  LadybugContext context;
  LadybugError error = ladybugCreateContext(&context);

  if(error != LADYBUG_OK) {
    std::cout << "Error creating context: " << ladybugErrorToString(error) << std::endl;
    return;
  }

  std::string file_path = __FILE__;
  std::string config_file = "/home/steve/dev_ws/src/core/ladybug_camera/.ladybug.conf";
  std::cout << "Loading config from: " << config_file << std::endl;
  error = ladybugLoadConfig(context, config_file.c_str());
  if(error != LADYBUG_OK) {
    std::cout << "Error loading config: " << ladybugErrorToString(error) << std::endl;
    return;
  }
  std::cout << "Loaded .conf file" << std::endl;
  // These (dummy) calls are required in order to obtain intrinsics
  error = ladybugConfigureOutputImages(context, LADYBUG_RECTIFIED_CAM0);
  error =
      ladybugSetOffScreenImageSize(context, LADYBUG_RECTIFIED_CAM0, 2464, 2048);

  std::cout << "Trying to generate weights" << std::endl;

  for(int i = 0; i < 2464; i++) {
    for(int j = 0; j < 2048; j++) {
      r_pixels[i * 2048 + j].first = 0; // = std::make_pair<int, int>(u, v);
      r_pixels[i * 2048 + j].second = 0;
      std::get<0>(weights[i * 2048 + j]) = 0;
      std::get<1>(weights[i * 2048 + j]) = 0;
      std::get<2>(weights[i * 2048 + j]) = 0;
      std::get<3>(weights[i * 2048 + j]) = 0;
    }
  }

  std::cout << "Trying to generate weights" << std::endl;
  int count = 0;
  for(int i = 0; i < 2464; ++i) {
    for(int j = 0; j < 2048; ++j) {
      double x, y;
      //          std::cout << i << ", " << j << "\n";

      error = ladybugUnrectifyPixel(context, cam_id_, j, i, &y, &x);
      if(error == LADYBUG_OK) {
        //          std::cout << x << ", " << y << "\n";
        int u = std::floor(x);
        int v = std::floor(y);
        //          if (u < 1 || v < 1) continue;
        count++;
        r_pixels[i * 2048 + j].first = u; // = std::make_pair<int, int>(u, v);
        r_pixels[i * 2048 + j].second = v;

        float A1 = ((u + 0.5) - (x - 0.5)) * ((v + 0.5) - (y - 0.5));
        float A2 = ((x + 0.5) - (u + 0.5)) * ((v + 0.5) - (y - 0.5));
        float A3 = ((x + 0.5) - (u + 0.5)) * ((y + 0.5) - (v + 0.5));
        float A4 = ((u + 0.5) - (x - 0.5)) * ((y + 0.5) - (v + 0.5));
        float total = A1 + A2 + A3 + A4;

        float w1 = A1 / total;
        float w2 = A2 / total;
        float w3 = A3 / total;
        float w4 = A4 / total;

        //          std::cout << "[" << i << ", " << j << "] maps to [" << x <<
        //          ", " << y << "]"
        //          << " rounded is [" << u << ", " << v << "]" << std::endl;
        //
        //          std::cout << w1 << "*[u,v] + "
        //                    << w2 << "*[u,v+1] + "
        //                    << w3 << "*[u+1,v+1] + "
        //                    << w4 << "*[u+1,v]" << std::endl;

        std::get<0>(weights[i * 2048 + j]) = w1;
        std::get<1>(weights[i * 2048 + j]) = w2;
        std::get<2>(weights[i * 2048 + j]) = w3;
        std::get<3>(weights[i * 2048 + j]) = w4;

      } else {
        std::cout << "Error: " << ladybugErrorToString(error) << std::endl;
        return;
      }
    }
  }
}

void LadybugRectifierNode::CreatePublishers() {

  // Monitor whether anyone is subscribed to the output
  using ConnectCb = image_transport::SubscriberStatusCallback;
  ConnectCb connect_cb = boost::bind(&LadybugRectifierNode::ConnectCb, this);

  // Make sure we don't enter ConnectCb() between advertising and assigning to
  // pub_XXX
  boost::lock_guard<boost::recursive_mutex> lock(mutex_);

  const auto istr = std::to_string(cam_id_);
  const std::string ns = "/ladybug/camera" + istr;
  const std::string topic = ns + "/rectified/image_color/";
  cam_pubs_[cam_id_] = img_trpt_.advertiseCamera(topic, 1, connect_cb, connect_cb);

}

// Handles (un)subscribing when clients (un)subscribe
void LadybugRectifierNode::ConnectCb() {

  boost::lock_guard<boost::recursive_mutex> lock(mutex_);
}

void LadybugRectifierNode::ImageCb(const sensor_msgs::ImageConstPtr& raw_msg) {

  boost::lock_guard<boost::recursive_mutex> lock(mutex_);

  auto cv_ptr =
      cv_bridge::toCvShare(raw_msg, sensor_msgs::image_encodings::BGR8);

  rect_img.forEach<cv::Vec3b>(
      [this, &cv_ptr](cv::Vec3b& pixel, const int* position) -> void {
        int u = position[0];
        int v = position[1];
        auto dist_u = r_pixels[u * 2048 + v].first;
        auto dist_v = r_pixels[u * 2048 + v].second;
        if(dist_u < 1 || dist_v < 1 || dist_u > 2462 || dist_v > 2046)
          return;
        else {

          float w1 = std::get<0>(weights[u * 2048 + v]);
          float w2 = std::get<1>(weights[u * 2048 + v]);
          float w3 = std::get<2>(weights[u * 2048 + v]);
          float w4 = std::get<3>(weights[u * 2048 + v]);

          cv::Vec3b rgb1 = cv_ptr->image.at<cv::Vec3b>(dist_u, dist_v);
          cv::Vec3b rgb2 = cv_ptr->image.at<cv::Vec3b>(dist_u, dist_v + 1);
          cv::Vec3b rgb3 = cv_ptr->image.at<cv::Vec3b>(dist_u + 1, dist_v + 1);
          cv::Vec3b rgb4 = cv_ptr->image.at<cv::Vec3b>(dist_u + 1, dist_v);

          pixel = w1 * rgb1 + w2 * rgb2 + w3 * rgb3 + w4 * rgb4;
        }
      }

  );

  rect_ptr->image = rect_img;

  auto msg = rect_ptr->toImageMsg();

  msg->header = raw_msg->header;
  msg->encoding = raw_msg->encoding;
  msg->width = raw_msg->width;
  msg->height = raw_msg->height;

  // if we failed to load the configuration file, try to at least provide
  // the size info that we can get from the image.
  if(!cam_infos_[cam_id_]) {
    const std::string istr = std::to_string(cam_id_);
    const std::string ns = "/ladybug/camera" + istr + "/rectify";

    ros::NodeHandle cim_nh(ns);
    cam_infos_[cam_id_].reset(new camera_info_manager::CameraInfoManager(cim_nh));
    cam_infos_[cam_id_]->setCameraName(std::string("ladybug") + istr);

    sensor_msgs::CameraInfo cam_info;
    cam_info.header = raw_msg->header;
    cam_info.width = raw_msg->width;
    cam_info.height = raw_msg->height;
    cam_infos_[cam_id_]->setCameraInfo(cam_info);
  }

  auto ci = boost::make_shared<sensor_msgs::CameraInfo>(
      cam_infos_[cam_id_]->getCameraInfo());
  ci->header = raw_msg->header;
  cam_pubs_[cam_id_].publish(msg, ci);
}


} // namespace ladybug_camera
