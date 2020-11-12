#pragma once

#include "LadybugInterface.h"

// OpenCV
#include <opencv2/core/core.hpp>

/**
 * @brief Implementation of LadybugInterface for using camera in rectified mode
 * (i.e., publishing colored & rectified images)
 */
class LadybugCompressed : public LadybugInterface {
public:
  ~LadybugCompressed() = default;

  void Initialize() override;

  void GrabImage(LadybugTilesPtr& tiles,
                 const std::string& frame_id) override;

  void GrabImage(sensor_msgs::Image& img) override;
};