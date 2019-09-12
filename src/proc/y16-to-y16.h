#pragma once

#include "synthetic-stream.h"

namespace librealsense
{
    class y16_to_y16 : public stream_filter_processing_block
    {
    public:
        y16_to_y16();
        y16_to_y16(const char* name);
        rs2::frame process_frame(const rs2::frame_source& source, const rs2::frame& f);

    private:
        rs2::stream_profile _target_stream_profile;
        rs2::stream_profile _source_stream_profile;
        int _traget_bpp = 2;
    };
}
