// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#pragma once
#ifndef LIBREALSENSE_IMAGE_H
#define LIBREALSENSE_IMAGE_H

#include "types.h"

namespace librealsense
{
    void unpack_y8_y8_from_y8i(rs2_format dst_left_format, rs2_stream dst_left_stream, rs2_format dst_right_format, rs2_stream dst_right_stream, byte * const dest[], const byte * source, int width, int height, int actual_size);
    void unpack_rotated_optimized(rs2_format dst_format, rs2_stream dst_stream, byte * const dest[], const byte * source, int width, int height, int actual_size);
    void unpack_yuy2(rs2_format dst_format, rs2_stream dst_stream, byte * const d[], const byte * s, int w, int h, int actual_size);
    void unpack_uyvyc(rs2_format dst_format, rs2_stream dst_stream, byte * const d[], const byte * s, int w, int h, int actual_size);
    void unpack_rgb_from_bgr(rs2_format dst_format, rs2_stream dst_stream, byte * const d[], const byte * s, int w, int h, int actual_size);
    void unpack_bayer16(rs2_format dst_format, rs2_stream dst_stream, byte * const dest[], const byte * source, int width, int height, int actual_size);
    void unpack_mjpeg(rs2_format dst_format, rs2_stream dst_stream, byte * const dest[], const byte * source, int width, int height, int actual_size);
    void unpack_motion_axes(rs2_format dst_format, rs2_stream dst_stream, byte * const dest[], const byte * source, int width, int height, int actual_size);
    void unpack_w10(rs2_format dst_format, rs2_stream dst_stream, byte * const dest[], const byte * source, int width, int height, int actual_size);
    void unpack_y16_y16_from_y12i_10(rs2_format dst_left_format, rs2_stream dst_left_stream, rs2_format dst_right_format, rs2_stream dst_right_stream, byte * const dest[], const byte * source, int width, int height, int actual_size);
    void unpack_invi(rs2_format dst_format, rs2_stream dst_stream, byte * const d[], const byte * s, int width, int height, int actual_size);
    void unpack_inzi(rs2_format dst_left_format, rs2_stream dst_left_stream, rs2_format dst_right_format, rs2_stream dst_right_stream, byte * const d[], const byte * s, int width, int height, int actual_size);

    size_t           get_image_size                 (int width, int height, rs2_format format);
    int              get_image_bpp                  (rs2_format format);

    resolution rotate_resolution(resolution res);
    resolution l500_confidence_resolution(resolution res);
}

#endif
