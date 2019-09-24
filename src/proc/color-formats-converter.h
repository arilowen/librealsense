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
    class LRS_EXTENSION_API color_formats_converter : public stream_filter_processing_block
    {
    public:
        color_formats_converter(rs2_format source_format, rs2_format target_format);

    protected:
        color_formats_converter(const char* name, rs2_format source_format, rs2_format target_format);
        rs2::frame process_frame(const rs2::frame_source& source, const rs2::frame& f) override;

    private:
        rs2::stream_profile _target_stream_profile;
        rs2::stream_profile _source_stream_profile;
        rs2_format _source_format;
        rs2_format _target_format;
        int _target_bpp = 0;
    };
}