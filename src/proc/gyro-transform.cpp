// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include "gyro-transform.h"

#include "../include/librealsense2/hpp/rs_sensor.hpp"
#include "../include/librealsense2/hpp/rs_processing.hpp"

#include "context.h"
#include "environment.h"
#include "image.h"

namespace librealsense
{
    gyro_transform::gyro_transform()
        : gyro_transform("Gyro Transform")
    {}

    gyro_transform::gyro_transform(const char* name)
        : stream_filter_processing_block(name)
    {
        _stream_filter.stream = RS2_STREAM_GYRO;
        _stream_filter.format = RS2_FORMAT_MOTION_XYZ32F;
    }

    rs2::frame gyro_transform::process_frame(const rs2::frame_source& source, const rs2::frame& f)
    {
        auto p = f.get_profile();
        if (p.get() != _source_stream_profile.get())
        {
            _source_stream_profile = p;
            _target_stream_profile = p.clone(p.stream_type(), p.stream_index(), RS2_FORMAT_MOTION_XYZ32F);
        }

        int width = f.get_data_size();
        int height = 1;
        rs2::frame ret = source.allocate_motion_frame(_target_stream_profile, f, _traget_bpp,
            width, height, width * _traget_bpp, RS2_EXTENSION_MOTION_FRAME);

        byte* planes[1];
        planes[0] = (byte*)ret.get_data();

        unpack_gyroscope_axes(planes, (const byte*)f.get_data(), width, height, width * height * _traget_bpp);

        return ret;
    }
}

