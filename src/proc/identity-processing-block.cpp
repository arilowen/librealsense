#include "identity-processing-block.h"
#include "image.h"

librealsense::identity_processing_block::identity_processing_block() :
    identity_processing_block("Identity processing block")
{
}

librealsense::identity_processing_block::identity_processing_block(const char * name) :
    stream_filter_processing_block(name)
{
}

rs2::frame librealsense::identity_processing_block::process_frame(const rs2::frame_source & source, const rs2::frame & f)
{
    //auto p = f.get_profile();
    //rs2_extension frame_extension;
    //int w = 0, h = 0;
    //if (auto frm = f.as<rs2::video_frame>())
    //{
    //    frame_extension = RS2_EXTENSION_VIDEO_FRAME;
    //    w = frm.get_width();
    //    h = frm.get_height();
    //}
    //else if (auto frm = f.as<rs2::depth_frame>())
    //{
    //    frame_extension = RS2_EXTENSION_DEPTH_FRAME;
    //    w = frm.get_width();
    //    h = frm.get_height();
    //}

    //rs2::frame ret;
    //auto bpp = get_image_bpp(p.format());
    //ret = source.allocate_video_frame(p, f, bpp,
    //    w, h, w * bpp, frame_extension);

    return f;
}
