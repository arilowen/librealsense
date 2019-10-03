// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#pragma once

#include "synthetic-stream.h"

#include "ds5/ds5-motion.h"

namespace librealsense
{
    class LRS_EXTENSION_API motion_transform : public stream_filter_processing_block
    {
    public:
        motion_transform(rs2_format target_format, rs2_stream target_stream, mm_calib_handler* mm_calib = nullptr, bool is_motion_correction_enabled = false);

    protected:
        motion_transform(const char* name, rs2_format target_format, rs2_stream target_stream, mm_calib_handler* mm_calib, bool is_motion_correction_enabled);
        rs2::frame process_frame(const rs2::frame_source& source, const rs2::frame& f) override;

    private:
        void correct_motion(rs2::frame* f);

        rs2_format _target_format;
        rs2_stream _target_stream;

        rs2::stream_profile _target_stream_profile;
        rs2::stream_profile _source_stream_profile;

        int _traget_bpp = 1;

        mm_calib_handler* _mm_calib = nullptr;
        bool _is_motion_correction_enabled = false;
    };
}