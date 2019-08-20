#pragma once

#include "synthetic-stream.h"

namespace librealsense
{
    class LRS_EXTENSION_API y8i_to_y8y8 : public processing_block
    {
    public:
        y8i_to_y8y8(int left_idx = 1, int right_idx = 2);

    protected:
        y8i_to_y8y8(const char* name);

    private:
        void init();

        int _left_target_profile_idx = 1;
        int _right_target_profile_idx = 2;
        int _traget_bpp = 1;
    };
}