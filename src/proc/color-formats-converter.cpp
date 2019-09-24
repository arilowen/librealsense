// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include "color-formats-converter.h"

librealsense::color_formats_converter::color_formats_converter(rs2_format source_format, rs2_format target_format) : color_formats_converter("Color Formats Converter", source_format, target_format)
{};

librealsense::color_formats_converter::color_formats_converter(const char * name, rs2_format source_format, rs2_format target_format) : stream_filter_processing_block(name)
{
    _stream_filter.stream = RS2_STREAM_COLOR;
    _stream_filter.format = source_format;

    _source_format = source_format;
    _target_format = target_format;
};

rs2::frame librealsense::color_formats_converter::process_frame(const rs2::frame_source & source, const rs2::frame & f)
{
    auto p = f.get_profile();
    if (p.get() != _source_stream_profile.get())
    {
        _source_stream_profile = p;
        _target_stream_profile = p.clone(p.stream_type(), p.stream_index(), _target_format);
        _target_bpp = get_image_bpp(_target_format) / 8;
    }

    auto vf = f.as<rs2::video_frame>();
    rs2::frame ret = source.allocate_video_frame(_target_stream_profile, f, _target_bpp,
        vf.get_width(), vf.get_height(), vf.get_width() * _target_bpp, RS2_EXTENSION_VIDEO_FRAME);

    byte* planes[1];
    planes[0] = (byte*)ret.get_data();

    switch (_source_format)
    {
    case RS2_FORMAT_YUYV:
        unpack_yuy2_to(_target_format, planes, (const byte*)f.get_data(), vf.get_width(), vf.get_height(), vf.get_height() * vf.get_width() * _target_bpp);
        break;
    case RS2_FORMAT_UYVY:
        unpack_uyvyc_to(_target_format, planes, (const byte*)f.get_data(), vf.get_width(), vf.get_height(), vf.get_height() * vf.get_width() * _target_bpp);
        break;
    case RS2_FORMAT_RAW16:
        unpack_bayer16(planes, (const byte*)f.get_data(), vf.get_width(), vf.get_height(), vf.get_height() * vf.get_width() * _target_bpp);
        break;
    case RS2_FORMAT_MJPEG:
        unpack_mjpeg(planes, (const byte*)f.get_data(), vf.get_width(), vf.get_height(), vf.get_height() * vf.get_width() * _target_bpp);
        break;
    default:
        LOG_ERROR("Trying to convert unsupported source format :", rs2_format_to_string(_source_format));
        break;
    }

    return ret;
}
