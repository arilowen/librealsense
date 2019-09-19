// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include "acceleration-transform.h"

#include "../include/librealsense2/hpp/rs_sensor.hpp"
#include "../include/librealsense2/hpp/rs_processing.hpp"

#include "context.h"
#include "environment.h"
#include "image.h"

namespace librealsense
{
    acceleration_transform::acceleration_transform(frame_callback_ptr cb)
        : acceleration_transform("Acceleration Transform")
    {
        _callback = cb;
    }

    acceleration_transform::acceleration_transform(const char* name)
        : stream_filter_processing_block(name)
    {
        _stream_filter.stream = RS2_STREAM_ACCEL;
        _stream_filter.format = RS2_FORMAT_MOTION_XYZ32F;
    }

    rs2::frame acceleration_transform::process_frame(const rs2::frame_source& source, const rs2::frame& f)
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

        unpack_acceleration_axes(planes, (const byte*)f.get_data(), width, height, width * height * _traget_bpp);

        if (_callback)
            _callback->on_frame((rs2_frame*)ret.get());

        return ret;
    }

    motion_transform::motion_transform(rs2_format target_format, rs2_stream target_stream, frame_callback_ptr cb)
        : motion_transform("Motion Transform")
    {
        _callback = cb;
        _target_format = target_format;
        _target_stream = target_stream;
    }

    motion_transform::motion_transform(const char* name)
        : stream_filter_processing_block(name)
    {} 

    rs2::frame motion_transform::process_frame(const rs2::frame_source& source, const rs2::frame& f)
    {
        auto p = f.get_profile();
        if (p.get() != _source_stream_profile.get())
        {
            _stream_filter.stream = _target_stream;
            _stream_filter.format = _target_format;
            _source_stream_profile = p;
            _target_stream_profile = p.clone(p.stream_type(), p.stream_index(), _target_format);
        }

        int width = f.get_data_size();
        int height = 1;
        rs2::frame ret = source.allocate_motion_frame(_target_stream_profile, f, _traget_bpp,
            width, height, width * _traget_bpp, RS2_EXTENSION_MOTION_FRAME);

        byte* planes[1];
        planes[0] = (byte*)ret.get_data();

        if (_target_stream == RS2_STREAM_ACCEL)
            unpack_acceleration_axes(planes, (const byte*)f.get_data(), width, height, width * height * _traget_bpp);
        else if (_target_stream == RS2_STREAM_GYRO)
            unpack_gyroscope_axes(planes, (const byte*)f.get_data(), width, height, width * height * _traget_bpp);

        if (_callback)
            _callback->on_frame((rs2_frame*)ret.get());

        return ret;
    }
}

