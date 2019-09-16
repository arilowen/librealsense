// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#pragma once

#include "synthetic-stream.h"
#include "option.h"
#include "image.h"
#include "../include/librealsense2/hpp/rs_sensor.hpp"
#include "../include/librealsense2/hpp/rs_processing.hpp"

namespace librealsense
{
    template<rs2_format T>
    class LRS_EXTENSION_API yuy2_converter : public stream_filter_processing_block
    {
    public:
        yuy2_converter() : yuy2_converter("YUY Converter")
        {};

    protected:
        yuy2_converter(const char* name) : stream_filter_processing_block(name) {
            _stream_filter.stream = RS2_STREAM_ANY;
            _stream_filter.format = RS2_FORMAT_YUYV;
        };

        rs2::frame process_frame(const rs2::frame_source& source, const rs2::frame& f) override
        {
            auto p = f.get_profile();
            if (p.get() != _source_stream_profile.get())
            {
                _source_stream_profile = p;
                _target_stream_profile = p.clone(p.stream_type(), p.stream_index(), T);
                _traget_bpp = get_image_bpp(T) / 8;
            }

            auto vf = f.as<rs2::video_frame>();
            rs2::frame ret = source.allocate_video_frame(_target_stream_profile, f, _traget_bpp,
                vf.get_width(), vf.get_height(), vf.get_width() * _traget_bpp, RS2_EXTENSION_VIDEO_FRAME);

            byte* planes[1];
            planes[0] = (byte*)ret.get_data();

            unpack_yuy2_to(T, planes, (const byte*)f.get_data(), vf.get_width(), vf.get_height(), vf.get_height() * vf.get_width() * _traget_bpp);

            return ret;
        }

    private:
        rs2::stream_profile _target_stream_profile;
        rs2::stream_profile _source_stream_profile;
        int _traget_bpp = 0;
    };
}