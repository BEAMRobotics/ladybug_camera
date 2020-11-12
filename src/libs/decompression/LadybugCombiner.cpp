#include "ladybug_camera/decompression/LadybugCombiner.h"

#include <dc1394/conversions.h>
#include <ros/ros.h>

#include <fastjpeg/fastjpeg.h>
#include <sensor_msgs/image_encodings.h>

#include <boost/thread/thread.hpp>

namespace ladybug_camera {

LadybugCombiner::LadybugCombiner() {

  for(unsigned int i = 0; i < 24; ++i) {
    // Initialize the jpeg struct according to fastjpeg docs
    jpeg_struct_[i] = fastjpeg::init_decompress();

    // These are rotated, so h and w are flipped on purpose
    bayer_image_[i].width = LadybugCombiner::HALF_HEIGHT;
    bayer_image_[i].height = LadybugCombiner::HALF_WIDTH;
    bayer_image_[i].nchannels = 1;
    bayer_image_[i].pix.resize(bayer_image_[i].width * bayer_image_[i].height *
                               bayer_image_[i].nchannels);
  }

  for(auto& combined_image : combined_images_) {
    combined_image.resize(LadybugCombiner::FULL_HEIGHT *
                          LadybugCombiner::FULL_WIDTH);
  }
}

LadybugCombiner::~LadybugCombiner() {

  // Required according to fastjpeg docs
  for(unsigned int i = 0; i < 24; i++)
    fastjpeg::release(jpeg_struct_ + i);
}

std::vector<sensor_msgs::ImagePtr>
    LadybugCombiner::operator()(const LadybugTilesCompressed& raw_images) {

  std::vector<sensor_msgs::ImagePtr> bayer_images;
  Recombine(raw_images, bayer_images);

  return bayer_images;
}

// Decompress the images and place the pixels in the recombined image.
// Take care of camera that were excluded (size of data is 0)
bool LadybugCombiner::Decompress4(const LadybugTilesCompressed& raw_images,
                                  unsigned c4) {

  for(unsigned j = 0; j < 4; ++j) {
    const unsigned J = c4 + j;
    if(raw_images.images[J].data.empty())
      return false;

    fastjpeg::decompress_memory(jpeg_struct_[J],
                                &(raw_images.images[J].data[0]),
                                raw_images.images[J].data.size(),
                                bayer_image_ + J,
                                nullptr);
  }

  return true;
}

// Rotate and Combine the 4 images to create one image with the bayer
// pattern. The images come as RGGB from the sensor. However, since we are
// rotating the image, we are creating a GRBG pattern
void LadybugCombiner::Combine(unsigned c4, unsigned char* data) {

  // we will read the bayer images in their natural order
  for(unsigned y = 0; y < HALF_WIDTH; ++y) {
    for(unsigned x = 0; x < HALF_HEIGHT; ++x) {
      // index in the individual images
      const unsigned j = y * HALF_HEIGHT + x;

      // index in the bayer image
      //   *2 at the end is because we fill 2 rows and 2 cols at the time
      const unsigned i = (x * FULL_WIDTH + (HALF_WIDTH - 1 - y)) * 2;

      data[i] = bayer_image_[c4 + 2].pix[j];
      data[i + 1] = bayer_image_[c4 + 0].pix[j];
      data[i + FULL_WIDTH] = bayer_image_[c4 + 3].pix[j];
      data[i + FULL_WIDTH + 1] = bayer_image_[c4 + 1].pix[j];
    }
  }
}

void LadybugCombiner::Recombine(
    const LadybugTilesCompressed& raw_images,
    std::vector<sensor_msgs::ImagePtr>& bayer_images) {

  bayer_images.resize(6);

#pragma omp parallel for
  for(unsigned int cam = 0; cam < 6; ++cam) {

    const unsigned int c4 = cam * 4;
    if(!cam_enabled_[cam])
      continue;

    const bool valid_camera_img = Decompress4(raw_images, c4);

    // Prepare the output
    // Make sure the image is properly defined and allocated
    if(!valid_camera_img) {
      bayer_images[cam].reset();
      continue;
    }

    if(!bayer_images[cam])
      bayer_images[cam].reset(new sensor_msgs::Image);

    sensor_msgs::Image& img = *(bayer_images[cam]);
    img.header = raw_images.header;
    std::stringstream ss;
    ss << "/ladybug/camera" << cam;
    img.header.frame_id = ss.str();

    img.height = FULL_HEIGHT;
    img.width = FULL_WIDTH;
    img.is_bigendian = false;
    if(do_debayering_) {
      img.encoding = sensor_msgs::image_encodings::RGB8;
      img.step = img.width * 3;
    } else {
      img.encoding = sensor_msgs::image_encodings::BAYER_GRBG8;
      img.step = img.width;
    }
    img.data.resize(img.height * img.step);

    // select the output of the recombining stage
    std::vector<unsigned char>& data =
        do_debayering_ ? combined_images_[cam] : img.data;

    Combine(c4, &(data[0]));

    if(do_debayering_) {
      dc1394_bayer_decoding_8bit(&(combined_images_[cam][0]),
                                 &(img.data[0]),
                                 FULL_WIDTH,
                                 FULL_HEIGHT,
                                 DC1394_COLOR_FILTER_GRBG,
                                 debayer_alg_);
    }
  }
}

} // namespace ladybug_camera
