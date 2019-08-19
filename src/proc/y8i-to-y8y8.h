#pragma once

#include "synthetic-stream.h"

namespace librealsense
{
    class LRS_EXTENSION_API y8i_to_y8y8 : public stream_filter_processing_block
    {
    public:
        y8i_to_y8y8();

    protected:
        y8i_to_y8y8(const char* name);
        rs2::frame process_frame(const rs2::frame_source& source, const rs2::frame& f) override;

    private:
        rs2::stream_profile _target_stream_profile;
        rs2::stream_profile _source_stream_profile;
        int _traget_bpp = 1;
    };
}