#pragma once

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "ladybug_camera/driver/LadybugCompressed.h"
#include "ladybug_camera/driver/LadybugRaw.h"
#include "ladybug_camera/driver/LadybugRectified.h"
#include <image_transport/image_transport.h>


#include <boost/thread.hpp>
#include <fstream>

namespace ladybug_camera {

class LadybugDriverNodelet : public nodelet::Nodelet {
public:
  LadybugDriverNodelet() = default;

  ~LadybugDriverNodelet() override;

private:
  /**
   * Function called when nodelet is loaded. Grabs ROS parameters, initializes
   * dynamic reconfigure, initializes camera info manager
   */
  void onInit() override;

  /**
   * Function for the boost::thread to grabImages and publish them.
   * This function continues until the thread is interrupted. Responsible for
   * getting images and publishing them.
   */
  void DevicePoll();


  //! Group threading

  // For connecting to camera
  boost::mutex connect_mutex_;

  // The thread that reads and publishes the images.
  boost::shared_ptr<boost::thread> publish_thread_;

  std::unique_ptr<LadybugInterface> ladybug_;

  std::shared_ptr<ros::Publisher> img_packet_pub_;
//  ros::Publisher img_packet_pub_;
  ros::SubscriberStatusCallback status_callback_;
  void ConnectCallback();
};

PLUGINLIB_EXPORT_CLASS(ladybug_camera::LadybugDriverNodelet,
                       nodelet::Nodelet)
} // namespace ladybug_camera