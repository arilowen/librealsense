#include "y8i-to-y8y8.h"

#include "stream.h"
#include "context.h"
#include "image.h"

namespace librealsense
{

    y8i_to_y8y8::y8i_to_y8y8() : y8i_to_y8y8("Y8i to Y8-Y8 Converter")
    {}

    y8i_to_y8y8::y8i_to_y8y8(const char * name)
        : processing_block(name)
    {
        // define and set the frame processing callback
        auto process_callback = [&](frame_holder frame, synthetic_source_interface* source)
        {
            rs2::frame* f = (rs2::frame*)&frame;
            auto p = f->get_profile();
            if (p.get() != _source_stream_profile.get())
            {
                _source_stream_profile = p;
                _target_stream_profile = p.clone(p.stream_type(), p.stream_index(), RS2_FORMAT_Y8);
            }

            frame_holder l, r;
            //rs2::frame_source s = { (rs2_source*)source->get_c_wrapper() };
            auto target_stream_profile = dynamic_cast<video_stream_profile*>(_target_stream_profile.get()->profile);
            std::shared_ptr<video_stream_profile> tsp(target_stream_profile);
            auto vf = f->as<rs2::video_frame>();
            l = source->allocate_video_frame(tsp, frame, _traget_bpp,
                vf.get_width(), vf.get_height(), vf.get_width() * _traget_bpp, RS2_EXTENSION_VIDEO_FRAME);
            r = source->allocate_video_frame(tsp, frame, _traget_bpp,
                vf.get_width(), vf.get_height(), vf.get_width() * _traget_bpp, RS2_EXTENSION_VIDEO_FRAME);

            byte* planes[2];
            planes[0] = (byte*)l.frame->get_frame_data();
            planes[1] = (byte*)r.frame->get_frame_data();
            unpack_y8_y8_from_y8i(planes, (const byte*)frame->get_frame_data(), vf.get_width(), vf.get_height());

            /*frame_holder f;
            while (matches.try_dequeue(&f))*/
            source->frame_ready(std::move(l));
            source->frame_ready(std::move(r));
        };
        set_processing_callback(std::shared_ptr<rs2_frame_processor_callback>(
            new internal_frame_processor_callback<decltype(process_callback)>(process_callback)));
    }

} // namespace librealsense
