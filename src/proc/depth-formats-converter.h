// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#pragma once

#include "synthetic-stream.h"
#include "option.h"
#include "image.h"

namespace librealsense
{
    class inzi_converter : public interleaved_functional_processing_block
    {
    public:
        inzi_converter(rs2_format target_ir_format) :
            inzi_converter("INZI to depth and IR Transform", target_ir_format) {};

    protected:
        inzi_converter(const char* name, rs2_format target_ir_format);
    };

    class invi_converter : public functional_processing_block
    {
    public:
        invi_converter(rs2_format target_format) :
            invi_converter("INVI to IR Transform", target_format) {};

    protected:
        invi_converter(const char* name, rs2_format target_format) :
            functional_processing_block(name, target_format, RS2_STREAM_INFRARED, RS2_EXTENSION_VIDEO_FRAME) {};
        rs2::frame process_frame(const rs2::frame_source & source, const rs2::frame & f) override;
    };
}