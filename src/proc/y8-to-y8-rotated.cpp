#include "y8-to-y8-rotated.h"

#include "../include/librealsense2/hpp/rs_sensor.hpp"
#include "../include/librealsense2/hpp/rs_processing.hpp"
#include "context.h"
#include "option.h"
#include "image.h"

namespace librealsense
{
    y8_to_y8_rotated::y8_to_y8_rotated()
        : y8_to_y8_rotated("Y8 Rotate")
    {}

    y8_to_y8_rotated::y8_to_y8_rotated(const char* name)
        : stream_filter_processing_block(name)
    {
        _stream_filter.stream = RS2_STREAM_INFRARED;
        _stream_filter.format = RS2_FORMAT_Y8;
    }

    rs2::frame y8_to_y8_rotated::process_frame(const rs2::frame_source& source, const rs2::frame& f)
    {
        auto p = f.get_profile();
        if (p.get() != _source_stream_profile.get())
        {
            _source_stream_profile = p;
            _target_stream_profile = p.clone(p.stream_type(), p.stream_index(), RS2_FORMAT_Y8);
            //auto target_spi = p.get()->profile->clone();
            //target_spi->set_unique_id(p.unique_id());
            //target_spi->set_stream_index(p.stream_index());
            //target_spi->set_format(RS2_FORMAT_Y8);
            //target_spi->set_stream_type(p.stream_type());
            //auto a = target_spi->get_c_wrapper();
            //auto b = std::shared_ptr<rs2_stream_profile>(new rs2_stream_profile{ target_spi.get(), target_spi });
            //_target_stream_profile = rs2::stream_profile(b.get());
        }

        rs2::frame ret;

        auto vf = f.as<rs2::video_frame>();
        ret = source.allocate_video_frame(_target_stream_profile, f, _traget_bpp,
            vf.get_width(), vf.get_height(), vf.get_width() * _traget_bpp, RS2_EXTENSION_VIDEO_FRAME);

        byte* planes[1];
        planes[0] = (byte*)ret.get_data();

        // pass flipped width and height
        align_l500_y8_optimized(planes, (const byte*)f.get_data(), vf.get_height(), vf.get_width(), vf.get_height() * vf.get_width() * _traget_bpp);

        return ret;
    }
}