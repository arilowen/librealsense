#include "y16-to-y16.h"

#include "../include/librealsense2/hpp/rs_sensor.hpp"
#include "../include/librealsense2/hpp/rs_processing.hpp"
#include "context.h"
#include "option.h"
#include "image.h"

namespace librealsense
{
    y16_to_y16::y16_to_y16()
        : y16_to_y16("Y16 Converter")
    {}

    y16_to_y16::y16_to_y16(const char* name)
        : stream_filter_processing_block(name)
    {
        _stream_filter.stream = RS2_STREAM_INFRARED;
        _stream_filter.format = RS2_FORMAT_Y16;
    }

    rs2::frame y16_to_y16::process_frame(const rs2::frame_source& source, const rs2::frame& f)
    {
        auto p = f.get_profile();
        if (p.get() != _source_stream_profile.get())
        {
            _source_stream_profile = p;
            _target_stream_profile = p.clone(p.stream_type(), p.stream_index(), RS2_FORMAT_Y16);
        }

        rs2::frame ret;

        auto vf = f.as<rs2::video_frame>();
        ret = source.allocate_video_frame(_target_stream_profile, f, _traget_bpp,
            vf.get_width(), vf.get_height(), vf.get_width() * _traget_bpp, RS2_EXTENSION_VIDEO_FRAME);

        byte* planes[1];
        planes[0] = (byte*)ret.get_data();

        // pass flipped width and height
        unpack_y16_from_y16_10(planes, (const byte*)f.get_data(), vf.get_height(), vf.get_width(), vf.get_height() * vf.get_width() * _traget_bpp);

        return ret;
    }
}