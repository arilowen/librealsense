// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include "rotation-transform.h"

#include "../include/librealsense2/hpp/rs_sensor.hpp"
#include "../include/librealsense2/hpp/rs_processing.hpp"
#include "context.h"
#include "image.h"
#include "stream.h"

namespace librealsense
{
    rotation_transform::rotation_transform(rs2_format target_format, rs2_stream target_stream, rs2_extension extension_type)
        : rotation_transform("Rotation Transform", target_format, target_stream, extension_type)
    {}

    rotation_transform::rotation_transform(const char* name, rs2_format target_format, rs2_stream target_stream, rs2_extension extension_type)
        : functional_processing_block(name, target_format, target_stream, extension_type)
    {
        _stream_filter.format = _target_format;
        _stream_filter.stream = _target_stream;
    }

    void rotation_transform::init(const rs2::frame* f)
    {
        auto p = f->get_profile();
        if (p.get() != _source_stream_profile.get())
        {
            _source_stream_profile = p;
            _target_stream_profile = p.clone(p.stream_type(), p.stream_index(), _target_format);
            _target_bpp = get_image_bpp(_target_format) / 8;

            // Set the unique ID as the original frame.
            // The frames are piped through a syncer and must have the origin UID.
            auto target_spi = (stream_profile_interface*)_target_stream_profile.get()->profile;
            target_spi->set_unique_id(p.unique_id());
        }
    }

    rs2::frame rotation_transform::process_frame(const rs2::frame_source & source, const rs2::frame & f)
    {
        return pre_process_frame(source, f, unpack_rotated_optimized);
    }

    depth_rotation_transform::depth_rotation_transform() :
        depth_rotation_transform("Depth Rotation Transform")
    {}

    depth_rotation_transform::depth_rotation_transform(const char * name)
        : rotation_transform(name, RS2_FORMAT_Z16, RS2_STREAM_DEPTH, RS2_EXTENSION_DEPTH_FRAME)
    {}

    ir_rotation_transform::ir_rotation_transform() :
        ir_rotation_transform("IR Rotation Transform")
    {}

    ir_rotation_transform::ir_rotation_transform(const char * name)
        : rotation_transform(name, RS2_FORMAT_Y8, RS2_STREAM_INFRARED, RS2_EXTENSION_VIDEO_FRAME)
    {}

    confidence_rotation_transform::confidence_rotation_transform() :
        confidence_rotation_transform("Confidence Rotation Transform")
    {}

    confidence_rotation_transform::confidence_rotation_transform(const char * name)
        : rotation_transform(name, RS2_FORMAT_RAW8, RS2_STREAM_CONFIDENCE, RS2_EXTENSION_VIDEO_FRAME)
    {}
}
