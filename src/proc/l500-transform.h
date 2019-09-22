#pragma once

#include "synthetic-stream.h"

namespace librealsense
{
    // Processes L500 rotated frames.
    class l500_transform : public stream_filter_processing_block
    {
    public:
        l500_transform(rs2_format target_format);
        l500_transform(const char* name, rs2_format target_format);
        rs2::frame process_frame(const rs2::frame_source& source, const rs2::frame& f);

    private:
        rs2::stream_profile _target_stream_profile;
        rs2::stream_profile _source_stream_profile;
        rs2_format _target_format;
        rs2_extension _extension_type;
        int _traget_bpp = 0;
    };
}