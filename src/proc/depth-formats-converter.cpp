// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include "depth-formats-converter.h"

#include "stream.h"

namespace librealsense
{
    // INZI converter
    inzi_converter::inzi_converter(const char * name, rs2_format target_ir_format)
        : interleaved_functional_processing_block(name, RS2_FORMAT_INZI, RS2_FORMAT_Z16, RS2_STREAM_DEPTH, RS2_EXTENSION_DEPTH_FRAME, 0,
                                                                         target_ir_format, RS2_STREAM_INFRARED, RS2_EXTENSION_VIDEO_FRAME, 1)
    {
        configure_processing_callback(unpack_inzi);
    };

    // INVI converter
    rs2::frame invi_converter::process_frame(const rs2::frame_source & source, const rs2::frame & f)
    {
        return pre_process_frame(source, f, unpack_invi);
    }

    w10_converter::w10_converter(const char * name, const rs2_format& target_format) :
        functional_processing_block(name, target_format, RS2_STREAM_INFRARED, RS2_EXTENSION_VIDEO_FRAME) {}
    rs2::frame w10_converter::process_frame(const rs2::frame_source & source, const rs2::frame & f)
    {
        return pre_process_frame(source, f, unpack_w10);
    }
}
