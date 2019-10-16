// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include "color-formats-converter.h"

namespace librealsense {
    rs2::frame yuy2_converter::process_frame(const rs2::frame_source & source, const rs2::frame & f)
    {
        return pre_process_frame(source, f, unpack_yuy2);
    }

    rs2::frame uyvy_converter::process_frame(const rs2::frame_source & source, const rs2::frame & f)
    {
        return pre_process_frame(source, f, unpack_uyvyc);
    }

    rs2::frame raw16_converter::process_frame(const rs2::frame_source & source, const rs2::frame & f)
    {
        return pre_process_frame(source, f, unpack_bayer16);
    }

    rs2::frame mjpeg_converter::process_frame(const rs2::frame_source & source, const rs2::frame & f)
    {
        return pre_process_frame(source, f, unpack_mjpeg);
    }
    rs2::frame bgr_to_rgb::process_frame(const rs2::frame_source & source, const rs2::frame & f)
    {
        return pre_process_frame(source, f, unpack_rgb_from_bgr);
    }
}
