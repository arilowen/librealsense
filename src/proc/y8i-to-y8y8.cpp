// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include "y8i-to-y8y8.h"

#include "stream.h"
#include "context.h"
#include "image.h"

namespace librealsense
{
    y8i_to_y8y8::y8i_to_y8y8(int left_idx, int right_idx) :
        y8i_to_y8y8("Y8i to Y8-Y8 Converter", left_idx, right_idx) {}

    y8i_to_y8y8::y8i_to_y8y8(const char * name, int left_idx, int right_idx)
        : interleaved_functional_processing_block(name, RS2_FORMAT_Y8I, RS2_FORMAT_Y8, RS2_STREAM_INFRARED, RS2_EXTENSION_VIDEO_FRAME, 1,
                                                                        RS2_FORMAT_Y8, RS2_STREAM_INFRARED, RS2_EXTENSION_VIDEO_FRAME, 2)
    {
        configure_processing_callback(unpack_y8_y8_from_y8i);
    }
} // namespace librealsense
