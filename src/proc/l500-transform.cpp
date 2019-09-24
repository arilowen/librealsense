#include "l500-transform.h"

#include "../include/librealsense2/hpp/rs_sensor.hpp"
#include "../include/librealsense2/hpp/rs_processing.hpp"
#include "context.h"
#include "image.h"
#include "stream.h"

namespace librealsense
{
    l500_transform::l500_transform(rs2_format target_format)
        : l500_transform("L500 Transform", target_format)
    {}

    l500_transform::l500_transform(const char* name, rs2_format target_format)
        : stream_filter_processing_block(name)
    {
        _target_format = target_format;

        if (_target_format == RS2_FORMAT_Z16)
        {
            _stream_filter.stream = RS2_STREAM_DEPTH;
            _extension_type =RS2_EXTENSION_DEPTH_FRAME;
        }
        else if (_target_format == RS2_FORMAT_Y8)
        {
            _stream_filter.stream = RS2_STREAM_INFRARED;
            _extension_type = RS2_EXTENSION_VIDEO_FRAME;
        }
        else if (_target_format == RS2_FORMAT_RAW8)
        {
            _stream_filter.stream = RS2_STREAM_CONFIDENCE;
            _extension_type = RS2_EXTENSION_VIDEO_FRAME;
        }
        _stream_filter.format = target_format;
    }

    rs2::frame l500_transform::process_frame(const rs2::frame_source& source, const rs2::frame& f)
    {
        auto p = f.get_profile();
        if (p.get() != _source_stream_profile.get())
        {
            _source_stream_profile = p;
            _target_stream_profile = p.clone(p.stream_type(), p.stream_index(), _target_format);

            // Set the unique ID as the original frame.
            // The frames are piped through a syncer and must have the origin UID.
            auto target_spi = (stream_profile_interface*)_target_stream_profile.get()->profile;
            target_spi->set_unique_id(p.unique_id());
        }

        auto vf = f.as<rs2::video_frame>();
        rs2::frame ret = source.allocate_video_frame(_target_stream_profile, f, _traget_bpp,
            vf.get_width(), vf.get_height(), vf.get_width() * _traget_bpp, _extension_type);

        byte* planes[1];
        planes[0] = (byte*)ret.get_data();

        auto vsp = As<video_stream_profile, stream_profile_interface>(p.get()->profile);
        auto bep = vsp->get_backend_profile();

        if (_target_format == RS2_FORMAT_Z16)
        {
            unpack_l500_z16_optimized(planes, (const byte*)f.get_data(), vf.get_height(), vf.get_width(), vf.get_height() * vf.get_width() * _traget_bpp);
        }
        else if (_target_format == RS2_FORMAT_Y8)
        {
            unpack_l500_y8_optimized(planes, (const byte*)f.get_data(), vf.get_height(), vf.get_width(), vf.get_height() * vf.get_width() * _traget_bpp);
        }
        else if (_target_format == RS2_FORMAT_RAW8)
        {
            unpack_confidence(planes, (const byte*)f.get_data(), bep.width, bep.height, 0);
        }

        return ret;
    }
}
