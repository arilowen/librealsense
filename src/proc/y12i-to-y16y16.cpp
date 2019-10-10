// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include "y12i-to-y16y16.h"

#include "stream.h"

namespace librealsense
{
    y12i_to_y16y16::y12i_to_y16y16(int left_idx, int right_idx)
        : y12i_to_y16y16("Y12I to Y16L Y16R Transform", left_idx, right_idx) {};

    y12i_to_y16y16::y12i_to_y16y16(const char * name, int left_idx, int right_idx)
        : interleaved_functional_processing_block(name, RS2_FORMAT_Y12I, RS2_FORMAT_Y16, RS2_STREAM_INFRARED, RS2_EXTENSION_VIDEO_FRAME, 1, 2)
    {
        configure_processing_callback(unpack_y16_y16_from_y12i_10);
    }
}