#include "y8i-to-y8y8.h"

#include "stream.h"
#include "context.h"
#include "image.h"

namespace librealsense
{
    y8i_to_y8y8::y8i_to_y8y8(int left_idx, int right_idx) :
        processing_block("Y8i to Y8-Y8 Converter"),
        _left_target_profile_idx(left_idx), _right_target_profile_idx(right_idx)
    {
        init();
    }

    y8i_to_y8y8::y8i_to_y8y8(const char * name)
        : processing_block(name)
    {
        init();
    }

    void y8i_to_y8y8::init()
    {
        // define and set the frame processing callback
        auto process_callback = [&](frame_holder frame, synthetic_source_interface* source)
        {
            auto profile = As<video_stream_profile, stream_profile_interface>(frame.frame->get_stream());
            auto lp = std::make_shared<video_stream_profile>(profile->get_backend_profile());
            auto rp = std::make_shared<video_stream_profile>(profile->get_backend_profile());
            auto w = profile->get_width();
            auto h = profile->get_height();

            // passthrough the frame if we don't need to process it.
            auto format = profile->get_format();
            if (format != RS2_FORMAT_Y8I)
            {
                auto bpp = get_image_bpp(format) / 8;
                auto extension = format == RS2_FORMAT_Z16 ? RS2_EXTENSION_DEPTH_FRAME : RS2_EXTENSION_VIDEO_FRAME;
                frame_holder frame_cpy = source->allocate_video_frame(profile, frame, bpp,
                    w, h, w * bpp, extension);
                memcpy((void*)frame_cpy->get_frame_data(), frame->get_frame_data(), sizeof(byte)*frame->get_frame_data_size());
                source->frame_ready(std::move(frame_cpy));
                //source->frame_ready(std::move(frame));
                return;
            }

            // process the frame
            lp->set_format(RS2_FORMAT_Y8);
            rp->set_format(RS2_FORMAT_Y8);
            lp->set_stream_type(profile->get_stream_type());
            rp->set_stream_type(profile->get_stream_type());
            lp->set_stream_index(_left_target_profile_idx);
            lp->set_unique_id(_left_target_profile_idx);
            rp->set_stream_index(_right_target_profile_idx);
            rp->set_unique_id(_right_target_profile_idx);

            frame_holder lf, rf;

            lf = source->allocate_video_frame(lp, frame, _traget_bpp,
                w, h, w * _traget_bpp, RS2_EXTENSION_VIDEO_FRAME);
            rf = source->allocate_video_frame(rp, frame, _traget_bpp,
                w, h, w * _traget_bpp, RS2_EXTENSION_VIDEO_FRAME);

            byte* planes[2];
            planes[0] = (byte*)lf.frame->get_frame_data();
            planes[1] = (byte*)rf.frame->get_frame_data();
            unpack_y8_y8_from_y8i(planes, (const byte*)frame->get_frame_data(), w, h, 0);

            source->frame_ready(std::move(lf));
            source->frame_ready(std::move(rf));
        };

        set_processing_callback(std::shared_ptr<rs2_frame_processor_callback>(
            new internal_frame_processor_callback<decltype(process_callback)>(process_callback)));
    }

} // namespace librealsense
