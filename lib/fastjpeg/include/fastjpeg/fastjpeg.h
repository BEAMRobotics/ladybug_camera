#ifndef FASTJPEG_H
#define FASTJPEG_H

#include <inttypes.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <jpeglib.h>


namespace fastjpeg
{

struct Image
{
  std::vector<uint8_t> pix;
  int width, height;
  int nchannels;
};

struct jpeg_decompress_struct* init_decompress();
struct jpeg_compress_struct* init_compress();
void release( struct jpeg_decompress_struct **cinfo );

/** Decompresses the data in jpeg_data
  *
  * - If dest_img is provided, it will be used to store the decompressed image.
  * Otherwise the destination image will be allocated.
  * - If histogram is provided, it will contain the histogram of values
  * (TODO: how should it be allocated, and what does the histogram mean)
  *
  * Returns the decompressed image.
  */
Image* decompress_memory( struct jpeg_decompress_struct *cinfo,
                          const unsigned char *jpeg_data, unsigned jpeg_size,
                          Image *dest_img=0, int *histogram=0 );

Image* decompress_file( jpeg_decompress_struct *cinfo, FILE *input_file,
                        Image* dest_img, int n_output_channels );

/**
  * Quality is from 5..95
  **/
void compress_file( jpeg_compress_struct *cinfo, Image* src_img,
                    const std::string& output_filename, int quality );
  
} //namespace fastjpeg

#endif //FASTJPEG_H
