#include "ladybug_camera/driver/LadybugDriverNodelet.h"


namespace ladybug_camera {

void LadybugDriverNodelet::onInit() {

  // Get nodeHandles
  ros::NodeHandle& nh = getMTNodeHandle();
  ros::NodeHandle& nh_priv = getMTPrivateNodeHandle();

  status_callback_ = boost::bind(&LadybugDriverNodelet::ConnectCallback, this);

  std::string topic = {};
  nh_priv.param<std::string>("topic", topic, "/ladybug/packet");

  img_packet_pub_.reset(new ros::Publisher(nh.advertise<sensor_msgs::Image>(topic, 1)));

  // ROS params
  bool enable_debug_logging_;
  nh_priv.param<bool>("enable_debug_logging", enable_debug_logging_, true);
  if(enable_debug_logging_) {
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                      ros::console::Level::Debug)) {
      ROS_INFO("Log level set to Debug");
      ros::console::notifyLoggerLevelsChanged();
    }
  }

  std::string camera_mode = {};
  nh_priv.param<std::string>("camera_mode", camera_mode, "");
  if(camera_mode == "raw") {
    ladybug_ = std::unique_ptr<LadybugInterface>(new LadybugRaw());
    ROS_DEBUG("Using ladybug in raw mode");
  } else if(camera_mode == "rectified") {
    ladybug_ = std::unique_ptr<LadybugInterface>(new LadybugRectified());
    ROS_DEBUG("Using ladybug in rectified mode");
  } else if(camera_mode == "compressed") {
    ladybug_ = std::unique_ptr<LadybugInterface>(new LadybugCompressed());
    ROS_DEBUG("Using ladybug in compressed mode");
  } else {
    throw std::runtime_error("Undefined camera mode!");
  }

  publish_thread_.reset(new boost::thread(boost::bind(&LadybugDriverNodelet::DevicePoll, this)));
}

void LadybugDriverNodelet::ConnectCallback(){
  ROS_INFO("Connect callback");
  if (img_packet_pub_->getNumSubscribers() != 0 && !publish_thread_){
    publish_thread_.reset(new boost::thread(boost::bind(&LadybugDriverNodelet::DevicePoll, this)));
  }
}

void LadybugDriverNodelet::DevicePoll() {
  enum State { NONE, ERROR, STOPPED, DISCONNECTED, CONNECTED, STARTED };

  State state = DISCONNECTED;
  State previous_state = NONE;

  while(!boost::this_thread::interruption_requested()) // Block until we need
                                                       // to stop this thread.
  {
    bool state_changed = (state != previous_state);

    previous_state = state;

    switch(state) {
      case ERROR:
        // Generally there's no need to stop before disconnecting after an
        // error. Indeed, stop will usually fail.
#if STOP_ON_ERROR
        // Try stopping the camera
        {
          boost::mutex::scoped_lock scopedLock(connect_mutex_);
          img_subs_.shutdown();
        }
        try {
          NODELET_INFO("Stopping camera.");
          ladybug_->stop();
          NODELET_INFO("Stopped camera.");

          state = STOPPED;
        } catch(std::runtime_error& e) {
          NODELET_ERROR_COND(
              state_changed, "Failed to stop error: %s", e.what());
          ros::Duration(1.0).sleep(); // sleep for one second each time
        }
        break;
#endif
      case STOPPED:
        // Try disconnecting from the camera
        try {
          NODELET_INFO("Disconnecting from camera.");
          ladybug_->Disconnect();
          NODELET_INFO("Disconnected from camera.");
          state = DISCONNECTED;
        } catch(std::runtime_error& e) {
          NODELET_ERROR_COND(
              state_changed, "Failed to disconnect with error: %s", e.what());
          ros::Duration(1.0).sleep(); // sleep for one second each time
        }
        break;
      case DISCONNECTED:
        // Try connecting to the camera
        try {
          ROS_INFO("Connecting to camera.");
          ladybug_->Connect();
          ROS_INFO("Connected to camera.");

          // Set last configuration, forcing the reconfigure level to stop
          // ladybug_->SetNewConfiguration(config_,
          // LadybugCamera::LEVEL_RECONFIGURE_STOP);

          // Set the timeout for grabbing images.
          try {
            double timeout;
            getMTPrivateNodeHandle().param("timeout", timeout, 1.0);
            NODELET_INFO("Setting timeout to: %f.", timeout);
            // ladybug_->setTimeout(timeout);
          } catch(std::runtime_error& e) { NODELET_ERROR("%s", e.what()); }
          state = CONNECTED;
        } catch(std::runtime_error& e) {
          NODELET_ERROR_COND(
              state_changed, "Failed to connect with error: %s", e.what());
          ros::Duration(1.0).sleep(); // sleep for one second each time
        }

        break;
      case CONNECTED:
        // Try starting the camera
        try {
          NODELET_INFO("Starting camera.");
          ladybug_->Start();
          NODELET_INFO("Started camera.");
          NODELET_INFO("If nothing subscribes to the camera topic, the "
                       "camera_info is not published .");
          state = STARTED;
        } catch(std::runtime_error& e) {
          NODELET_ERROR_COND(
              state_changed, "Failed to start with error: %s", e.what());
          ros::Duration(1.0).sleep(); // sleep for one second each time
        }
        break;
      case STARTED:
        try {
//          ROS_DEBUG("Trying to grab image & publish");

          sensor_msgs::Image image;
          ladybug_->GrabImage(image);
          img_packet_pub_->publish(image);

        } catch(CameraTimeoutException& e) {
          NODELET_WARN("%s", e.what());
        } catch(CameraImageConsistencyError& e) {
          NODELET_WARN("%s", e.what());
        } catch(std::runtime_error& e) {
          NODELET_ERROR("%s", e.what());
          state = ERROR;
        }
        break;
      default: NODELET_ERROR("Unknown camera state %d!", state);
    }
  }
  NODELET_INFO("Leaving thread.");
}

LadybugDriverNodelet::~LadybugDriverNodelet() {
  boost::mutex::scoped_lock scopedLock(connect_mutex_);

  if(publish_thread_) {
    publish_thread_->interrupt();
    publish_thread_->join();

    try {
      NODELET_INFO("Stopping camera capture.");
      ladybug_->Stop();
      NODELET_INFO("Disconnecting from camera.");
      ladybug_->Disconnect();
    } catch(std::runtime_error& e) { NODELET_ERROR("%s", e.what()); }
  }
}

} // namespace ladybug_camera
