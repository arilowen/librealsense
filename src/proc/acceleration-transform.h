// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#pragma once

#include "synthetic-stream.h"

namespace librealsense
{
    class LRS_EXTENSION_API acceleration_transform : public stream_filter_processing_block
    {
    public:
        acceleration_transform(frame_callback_ptr cb = nullptr);

    protected:
        acceleration_transform(const char* name);
        rs2::frame process_frame(const rs2::frame_source& source, const rs2::frame& f) override;
        frame_callback_ptr _callback;

    private:
        rs2::stream_profile _target_stream_profile;
        rs2::stream_profile _source_stream_profile;
        int _traget_bpp = 1;
    };
}