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
    class yuy2_converter : public color_processing_block
    {
    public:
        yuy2_converter(rs2_format target_format) :
            yuy2_converter("YUY Converter", target_format) {};

    protected:
        yuy2_converter(const char* name, rs2_format target_format) :
            color_processing_block(name, target_format) {};
        rs2::frame process_frame(const rs2::frame_source& source, const rs2::frame& f) override;
    };

    class uyvy_converter : public color_processing_block
    {
    public:
        uyvy_converter(rs2_format target_format) :
            uyvy_converter("UYVY Converter", target_format) {};

    protected:
        uyvy_converter(const char* name, rs2_format target_format) :
            color_processing_block(name, target_format) {};
        rs2::frame process_frame(const rs2::frame_source& source, const rs2::frame& f) override;
    };

    class mjpeg_converter : public color_processing_block
    {
    public:
        mjpeg_converter(rs2_format target_format) :
            mjpeg_converter("MJPEG Converter", target_format) {};

    protected:
        mjpeg_converter(const char* name, rs2_format target_format) :
            color_processing_block(name, target_format) {};
        rs2::frame process_frame(const rs2::frame_source& source, const rs2::frame& f) override;
    };

    class raw16_converter : public color_processing_block
    {
    public:
        raw16_converter(rs2_format target_format) :
            raw16_converter("RAW16 Converter", target_format) {};

    protected:
        raw16_converter(const char* name, rs2_format target_format) :
            color_processing_block(name, target_format) {};
        rs2::frame process_frame(const rs2::frame_source& source, const rs2::frame& f) override;
    };
}