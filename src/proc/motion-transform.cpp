// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include "motion-transform.h"

#include "../include/librealsense2/hpp/rs_sensor.hpp"
#include "../include/librealsense2/hpp/rs_processing.hpp"

#include "context.h"
#include "environment.h"
#include "image.h"

namespace librealsense
{
    motion_transform::motion_transform(rs2_format target_format, rs2_stream target_stream, mm_calib_handler* mm_calib, bool is_motion_correction_enabled)
        : motion_transform("Motion Transform", target_format, target_stream, mm_calib, is_motion_correction_enabled)
    {}

    motion_transform::motion_transform(const char* name, rs2_format target_format, rs2_stream target_stream, mm_calib_handler* mm_calib, bool is_motion_correction_enabled)
        : stream_filter_processing_block(name),
        _mm_calib(mm_calib),
        _is_motion_correction_enabled(is_motion_correction_enabled),
        _target_format(target_format),
        _target_stream(target_stream)
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
        rs2::frame ret = source.allocate_motion_frame(_target_stream_profile, f,
            width, height, RS2_EXTENSION_MOTION_FRAME);

        byte* planes[1];
        planes[0] = (byte*)ret.get_data();

        if (_target_stream == RS2_STREAM_ACCEL)
            unpack_acceleration_axes(planes, (const byte*)f.get_data(), width, height, width * height * _traget_bpp);
        else if (_target_stream == RS2_STREAM_GYRO)
            unpack_gyroscope_axes(planes, (const byte*)f.get_data(), width, height, width * height * _traget_bpp);

        correct_motion(&ret);

        return ret;
    }

    void motion_transform::correct_motion(rs2::frame* f)
    {
        if (!_mm_calib)
            return;

        auto xyz = (float3*)(f->get_data());

        try
        {
            auto accel_intrinsic = _mm_calib->get_intrinsic(RS2_STREAM_ACCEL);
            auto gyro_intrinsic = _mm_calib->get_intrinsic(RS2_STREAM_GYRO);

            if (_is_motion_correction_enabled)
            {
                auto&& s = f->get_profile().stream_type();
                if (s == RS2_STREAM_ACCEL)
                    *xyz = (accel_intrinsic.sensitivity * (*xyz)) - accel_intrinsic.bias;

                if (s == RS2_STREAM_GYRO)
                    *xyz = gyro_intrinsic.sensitivity * (*xyz) - gyro_intrinsic.bias;
            }
        }
        catch (const std::exception& ex)
        {
            LOG_INFO("Motion Module - no intrinsic calibration, " << ex.what());
        }

        // The IMU sensor orientation shall be aligned with depth sensor's coordinate system
        *xyz = _mm_calib->imu_to_depth_alignment() * (*xyz);
    }
}

