#pragma once

#include "synthetic-stream.h"

namespace librealsense
{
    class z16_to_z16_rotated : public stream_filter_processing_block
    {
    public:
        z16_to_z16_rotated();
        z16_to_z16_rotated(const char* name);
        rs2::frame z16_to_z16_rotated::process_frame(const rs2::frame_source& source, const rs2::frame& f);

    private:
        rs2::stream_profile _target_stream_profile;
        rs2::stream_profile _source_stream_profile;
        int _traget_bpp = 2;
    };
}