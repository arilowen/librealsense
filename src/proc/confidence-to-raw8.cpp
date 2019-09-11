#include "confidence-to-raw8.h"

#include "../include/librealsense2/hpp/rs_sensor.hpp"
#include "../include/librealsense2/hpp/rs_processing.hpp"
#include "image.h"
#include "context.h"
#include "stream.h"

namespace librealsense
{
    confidence_to_raw8::confidence_to_raw8()
        : confidence_to_raw8("Confidence to Raw8")
    {}

    confidence_to_raw8::confidence_to_raw8(const char* name)
        : stream_filter_processing_block(name)
    {
        _stream_filter.stream = RS2_STREAM_CONFIDENCE;
        _stream_filter.format = RS2_FORMAT_RAW8;
    }

    rs2::frame confidence_to_raw8::process_frame(const rs2::frame_source& source, const rs2::frame& f)
    {
        auto p = f.get_profile();
        if (p.get() != _source_stream_profile.get())
        {
            _source_stream_profile = p;
            _target_stream_profile = p.clone(p.stream_type(), p.stream_index(), RS2_FORMAT_RAW8);
        }

        rs2::frame ret;

        auto vf = f.as<rs2::video_frame>();
        ret = source.allocate_video_frame(_target_stream_profile, f, _traget_bpp,
            vf.get_width(), vf.get_height(), vf.get_width() * _traget_bpp, RS2_EXTENSION_VIDEO_FRAME);

        byte* planes[1];
        planes[0] = (byte*)ret.get_data();

        auto vsp = As<video_stream_profile, stream_profile_interface>(p.get()->profile);
        auto bep = vsp->get_backend_profile();
        //auto w = 
        // pass flipped width and height
        unpack_confidence(planes, (const byte*)f.get_data(), bep.width, bep.height, 0);

        return ret;
    }
}
