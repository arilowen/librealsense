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
        rotation_transform(rs2_format target_format, rs2_stream target_stream, rs2_extension extension_type);
        rotation_transform(const char* name, rs2_format target_format, rs2_stream target_stream, rs2_extension extension_type);

    protected:
        void init(const rs2::frame* f) override;
        rs2::frame process_frame(const rs2::frame_source& source, const rs2::frame& f) override;
    };

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
