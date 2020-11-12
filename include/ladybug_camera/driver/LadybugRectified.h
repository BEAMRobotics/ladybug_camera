#pragma once

#include "LadybugInterface.h"

// OpenCV
#include <opencv2/core/core.hpp>

/**
 * @brief Implementation of LadybugInterface for using camera in rectified mode
 * (i.e., publishing colored & rectified images)
 */
class LadybugRectified : public LadybugInterface {
public:
  ~LadybugRectified() = default;

  void Initialize() override;

  void GrabImage(LadybugTilesPtr& tiles,
                 const std::string& frame_id) override;
};