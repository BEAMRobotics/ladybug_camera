#include "ladybug_camera/driver/LadybugCompressed.h"
#include <thread>
#include <atomic>

void LadybugCompressed::Initialize() {
  ROS_DEBUG("[LadybugCompressed] Initializing camera...");

  COLOR_PROCESSING_METHOD =
      LadybugColorProcessingMethod::LADYBUG_DISABLE; // LADYBUG_DOWNSAMPLE4;
  PROCESSED_PIXEL_FORMAT = LadybugPixelFormat::LADYBUG_BGR;
  LADYBUG_DATA_FORMAT = LadybugDataFormat::LADYBUG_DATAFORMAT_COLOR_SEP_JPEG8;
//  LADYBUG_DATA_FORMAT = LadybugDataFormat::LADYBUG_DATAFORMAT_COLOR_SEP_HALF_HEIGHT_JPEG8;

  // Set color processing method
  error_ = ladybugSetColorProcessingMethod(context_, COLOR_PROCESSING_METHOD);
  ROS_DEBUG("ladybugSetColorProcessingMethod: %s",
            ladybugErrorToString(error_));

  // Set frame rate
  error_ = ladybugSetAbsProperty(
      context_, LadybugProperty::LADYBUG_FRAME_RATE, 60.0f);
  ROS_INFO("ladybugSetAbsProperty (Frame Rate): %s",
            ladybugErrorToString(error_));
  std::this_thread::sleep_for(std::chrono::seconds(2));


  GpsTimeSyncSettings timeSyncSettings;
  timeSyncSettings.baudRate = 9600;
  timeSyncSettings.enableGpsTimeSync = true;
  timeSyncSettings.enablePps = true;

  std::cout << std::endl  << "Setting GPSTimeSync properties on camera...";
  error_ = ladybugSetGpsTimeSync(context_, timeSyncSettings);
  if (error_ != LADYBUG_OK){
    std::cout << "FAILED!" << std::endl;
    return;
  } else std::cout << "Successful!" << std::endl;

  std::cout << "Validate GPSTimeSync properties were set..." << std::endl;
  GpsTimeSyncSettings retrievedSettings;
  error_ = ladybugGetGpsTimeSync(context_, retrievedSettings);
  if (error_ != LADYBUG_OK){
    return;
  }
  std::cout << "Baud Rate - " << retrievedSettings.baudRate << ", "
            << "GPSTimeSync Enabled: " << (retrievedSettings.enableGpsTimeSync ? "True" : "False")
            << ", PPS Enabled" << (retrievedSettings.enablePps ? "True" : "False") << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(6));

}

void LadybugCompressed::GrabImage(sensor_msgs::Image& img) {
  boost::mutex::scoped_lock scopedLock(mutex_);

  if(capture_running_) {

    // Grab image from camera, store in image_
    error_ = ladybugGrabImage(context_, &image_);
    if (error_ != LADYBUG_OK){
      ROS_WARN("Failed grabbing image: %s", ladybugErrorToString(error_));
      return;
    }
    ROS_DEBUG_THROTTLE(
        10, "ladybugGrabImage: %s", ladybugErrorToString(error_));
    /*std::cout << "Grabbed image" << std::endl;
    std::cout << "Image data size in bytes: " << image_.uiDataSizeBytes
              << std::endl;
    std::cout << "Image rows = " << image_.uiRows << std::endl;
    std::cout << "Image cols = " << image_.uiCols << std::endl;*/

    img.data.resize(image_.uiDataSizeBytes);
    memcpy(&img.data[0], image_.pData, image_.uiDataSizeBytes);


//    img.header.stamp = ros::Time::now();

    /*
    float value;
  error_ = ladybugGetAbsProperty(context_, LADYBUG_SHUTTER, &value);
  std::cout << "Shutter value = " << value << std::endl;
    std::cout << "image shutter time 0: " << (image_.imageInfo.ulShutter[0] & 0xfff) << std::endl;
    std::cout << "image shutter time 1: " << (image_.imageInfo.ulShutter[1] & 0xfff) << std::endl;
    std::cout << "image shutter time 2: " << (image_.imageInfo.ulShutter[2] & 0xfff) << std::endl;
    std::cout << "image shutter time 3: " << (image_.imageInfo.ulShutter[3] & 0xfff) << std::endl;
    std::cout << "image shutter time 4: " << (image_.imageInfo.ulShutter[4] & 0xfff)<< std::endl;
    std::cout << "image shutter time 5: " << (image_.imageInfo.ulShutter[5] & 0xfff) << std::endl;*/
//    error_ = ladybugSetAbsPropertyEx(
//        context_, LADYBUG_SHUTTER, false, true, false, 25.0f);
//    error_ = ladybugGetAbsProperty(context_, LADYBUG_SHUTTER, &value);
//    printf( "Shutter: %d, Gain: %d, \n",
//            image_.imageInfo.ulShutter[0] & 0xfff, image_.imageInfo.arulGainAdjust[0] & 0xfff);
/*

    float dRealShutter = 0.0;
    error_ = ladybugSetProperty( context_, LADYBUG_SHUTTER,image_.imageInfo.ulShutter[0] & 0xfff, 0, false);

    float fAbsValue;
    error_ = ladybugGetAbsProperty( context_, LADYBUG_SHUTTER, &fAbsValue);
    if ( error_ == LADYBUG_OK)
      dRealShutter = fAbsValue;
std::cout << "Actual shutter time: " << dRealShutter << std::endl;
*/


    img.header.frame_id = "ladybug";
    LadybugTimestamp timestamp = image_.timeStamp;
    img.header.stamp.sec = timestamp.ulSeconds;
    img.header.stamp.nsec = timestamp.ulMicroSeconds*1000;

    ROS_INFO_THROTTLE(10, "PPS Status: %ld, GPS Status: %ld", image_.imageInfo.bPpsStatus, image_.imageInfo.bGpsStatus);
    ROS_INFO_THROTTLE(10, "Seconds: %ld, Microseconds: %ld", image_.imageInfo.ulTimeSeconds, image_.imageInfo.ulTimeMicroSeconds);

    /*    std::cout << "   GPS status: " << image_.imageInfo.bGpsStatus << "\n"
              << "   PPS status: " << image_.imageInfo.bPpsStatus
              << std::endl;
    std::cout << "   Timestamp info...\n"
              << "      Seconds: " << timestamp.ulSeconds << "\n"
              << "      Microseconds: " << timestamp.ulMicroSeconds << "\n"
              << "      Cycle seconds: " << timestamp.ulCycleSeconds
              << std::endl;
    std::cout << "   Image info...\n"
              << "       Seconds (epoch): " << image_.imageInfo.ulTimeSeconds << "\n"
              << "       Microseconds: " << image_.imageInfo.ulTimeMicroSeconds
              << std::endl;*/
/*    auto time_seconds = static_cast<std::time_t>(timestamp.ulSeconds);
    std::cout << std::asctime(std::localtime(&time_seconds))
              << time_seconds << " seconds since the Epoch\n";*/


  } else if(camera_connected_) {
    throw CameraNotRunningException("LadybugCompressed::grabImage: Camera is "
                                    "currently not running.  Please "
                                    "start the capture.");
  } else {
    throw std::runtime_error("LadybugCompressed not connected!");
  }
//  ROS_DEBUG_THROTTLE(10, "Finished grabbing image");
}

void LadybugCompressed::GrabImage(LadybugTilesPtr& tiles,
                                  const std::string& frame_id) {
  boost::mutex::scoped_lock scopedLock(mutex_);

  return;
  if(capture_running_) {
    // Grab image from camera, store in image_
    error_ = ladybugGrabImage(context_, &image_);
    ROS_DEBUG_THROTTLE(
        10, "ladybugGrabImage: %s", ladybugErrorToString(error_));

    std::vector<LadybugOutputImage> ladybug_output_images = {
        LADYBUG_RECTIFIED_CAM0,
        LADYBUG_RECTIFIED_CAM1,
        LADYBUG_RECTIFIED_CAM2,
        LADYBUG_RECTIFIED_CAM3,
        LADYBUG_RECTIFIED_CAM4,
        LADYBUG_RECTIFIED_CAM5};

    // Set msg header information
    tiles->header.stamp.sec = image_.timeStamp.ulSeconds;
    tiles->header.stamp.nsec = image_.timeStamp.ulMicroSeconds;
    tiles->header.frame_id = frame_id;
    tiles->header.seq = image_.imageInfo.ulSequenceId;

    // Populate LadybugTiles message
    for(unsigned int cam = 0; cam < LADYBUG_NUM_CAMERAS; ++cam) {}
  } else if(camera_connected_) {
    throw CameraNotRunningException("LadybugCompressed::grabImage: Camera is "
                                    "currently not running.  Please "
                                    "start the capture.");
  } else {
    throw std::runtime_error("LadybugCompressed not connected!");
  }
  ROS_DEBUG_THROTTLE(10, "Finished grabbing image");
}
