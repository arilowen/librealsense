#include "y8i-to-y8y8.h"

#include "image.h"

namespace librealsense
{

    y8i_to_y8y8::y8i_to_y8y8() : y8i_to_y8y8("Y8i to Y8-Y8 Converter")
    {}

    y8i_to_y8y8::y8i_to_y8y8(const char * name)
        : stream_filter_processing_block(name)
    {
        _stream_filter.stream = RS2_STREAM_ANY;
        _stream_filter.format = RS2_FORMAT_Y8;
    }

    rs2::frame y8i_to_y8y8::process_frame(const rs2::frame_source & source, const rs2::frame & f)
    {
        auto p = f.get_profile();
        if (p.get() != _source_stream_profile.get())
        {
            _source_stream_profile = p;
            _target_stream_profile = p.clone(p.stream_type(), p.stream_index(), RS2_FORMAT_Y8);
        }

        rs2::frame ret;

        auto vf = f.as<rs2::video_frame>();
        ret = source.allocate_video_frame(_target_stream_profile, f, _traget_bpp,
            vf.get_width(), vf.get_height(), vf.get_width() * _traget_bpp, RS2_EXTENSION_VIDEO_FRAME);

        byte* planes[1];
        planes[0] = (byte*)ret.get_data();

        unpack_y8_y8_from_y8i(planes, (const byte*)f.get_data(), vf.get_width(), vf.get_height());

        return ret;
    }

} // namespace librealsense
