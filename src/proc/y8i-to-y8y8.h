#pragma once

#include "synthetic-stream.h"

namespace librealsense
{
    class LRS_EXTENSION_API y8i_to_y8y8 : public processing_block
    {
    public:
        y8i_to_y8y8();

    protected:
        y8i_to_y8y8(const char* name);

    private:
        rs2::stream_profile _target_stream_profile;
        rs2::stream_profile _source_stream_profile;
        int _traget_bpp = 1;
    };
}