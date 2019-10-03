// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#pragma once

#include "synthetic-stream.h"

namespace librealsense
{
    // Processes rotated frames.
    class rotation_transform : public functional_processing_block
    {
    public:
        rotation_transform(rs2_format target_format);
        rotation_transform(const char* name, rs2_format target_format);

    protected:
        void init(const rs2::frame* f) override;
        rs2::frame process_frame(const rs2::frame_source& source, const rs2::frame& f) override;
    };

    //class rotation_transform : public stream_filter_processing_block
    //{
    //public:
    //    rotation_transform(rs2_format target_format);
    //    rotation_transform(const char* name, rs2_format target_format);
    //    rs2::frame process_frame(const rs2::frame_source& source, const rs2::frame& f);

    //private:
    //    rs2::stream_profile _target_stream_profile;
    //    rs2::stream_profile _source_stream_profile;
    //    rs2_format _target_format;
    //    rs2_extension _extension_type;
    //    int _target_bpp = 0;
    //};

    class depth_rotation_transform : public rotation_transform
    {
    public:
        depth_rotation_transform();
        depth_rotation_transform(const char* name);
    };

    class ir_rotation_transform : public rotation_transform
    {
    public:
        ir_rotation_transform();
        ir_rotation_transform(const char* name);
    };

    class confidence_rotation_transform : public rotation_transform
    {
    public:
        confidence_rotation_transform();
        confidence_rotation_transform(const char* name);
    };
}
