#pragma once

#include "ladybug_camera/LadybugTilesCompressed.h"

#include <dc1394/conversions.h>

#include <sensor_msgs/Image.h>
#include <fastjpeg/fastjpeg.h>

namespace ladybug_camera {

/**
 * @brief Class for combining four independent image channels into
 * bayered image format.
 */
class LadybugCombiner {
public:
  LadybugCombiner();

  ~LadybugCombiner();

  /**
   * @brief Calls Recombine on raw_images, returns bayer images
   * @param raw_images
   * @return
   */
  std::vector<sensor_msgs::ImagePtr>
      operator()(const LadybugTilesCompressed& raw_images);

  /**
   * @brief Creates the raw bayer images
   * @details For each enabled camera, performs following image processing:
   *          1) Decompression of each of the 4 bayer channel images
   *          2) Rotates images
   *          3) Recombines decomp/rotated images into bayer image
   * @param raw_images The compressed / separated image streams
   * @return Vector of raw bayer images
   */
  void Recombine(const LadybugTilesCompressed& raw_images,
                 std::vector<sensor_msgs::ImagePtr>& bayer_images);

  // Width and height of lb image after all processing
  static constexpr unsigned int FULL_WIDTH = 2048;
  static constexpr unsigned int FULL_HEIGHT = 2464;

  bool do_debayering_ = false;
  dc1394bayer_method_t debayer_alg_ = DC1394_BAYER_METHOD_HQLINEAR;

  // cam_enabled_[i] = false -> ignore that camera
  bool cam_enabled_[6] = {true};

protected:
  // Half height / width of images
  static constexpr unsigned int HALF_HEIGHT = FULL_HEIGHT / 2;
  static constexpr unsigned int HALF_WIDTH = FULL_WIDTH / 2;

  // libjpeg "decompression instance" -> one for each channel
  jpeg_decompress_struct* jpeg_struct_[24]{};

  // fastjpeg image wrapper -> one for each channel
  fastjpeg::Image bayer_image_[24];

  std::vector<unsigned char> combined_images_[6];

  /**
   * @brief Decompresses 4 channels from raw images
   * @param raw_images
   * @param c4
   * @return
   */
  bool Decompress4(const LadybugTilesCompressed& raw_images, unsigned c4);

  /**
   * @brief Combine four decompressed channels
   * @param c4
   * @param data
   */
  void Combine(unsigned c4, unsigned char* data);
};

} // namespace ladybug_camera