// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#pragma once

#include "core/processing.h"
#include "image.h"
#include "source.h"
#include "../include/librealsense2/hpp/rs_frame.hpp"
#include "../include/librealsense2/hpp/rs_processing.hpp"

namespace librealsense
{
    class synthetic_source : public synthetic_source_interface
    {
    public:
        synthetic_source(frame_source& actual)
            : _actual_source(actual), _c_wrapper(new rs2_source{ this })
        {
        }

        frame_interface* allocate_video_frame(std::shared_ptr<stream_profile_interface> stream,
            frame_interface* original,
            int new_bpp = 0,
            int new_width = 0,
            int new_height = 0,
            int new_stride = 0,
            rs2_extension frame_type = RS2_EXTENSION_VIDEO_FRAME) override;

        frame_interface* allocate_motion_frame(std::shared_ptr<stream_profile_interface> stream,
            frame_interface* original,
            int width = 0,
            int height = 0,
            rs2_extension frame_type = RS2_EXTENSION_MOTION_FRAME) override;

        frame_interface* allocate_composite_frame(std::vector<frame_holder> frames) override;

        frame_interface* allocate_points(std::shared_ptr<stream_profile_interface> stream, 
            frame_interface* original, rs2_extension frame_type = RS2_EXTENSION_POINTS) override;

        void frame_ready(frame_holder result) override;

        rs2_source* get_c_wrapper() override { return _c_wrapper.get(); }

    private:
        frame_source & _actual_source;
        std::shared_ptr<rs2_source> _c_wrapper;
    };

    class LRS_EXTENSION_API processing_block : public processing_block_interface, public options_container, public info_container
    {
    public:
        processing_block(const char* name);

        void set_processing_callback(frame_processor_callback_ptr callback) override;
        void set_output_callback(frame_callback_ptr callback) override;
        void invoke(frame_holder frames) override;
        synthetic_source_interface& get_source() override { return _source_wrapper; }

        virtual ~processing_block() { _source.flush(); }
    protected:
        frame_source _source;
        std::mutex _mutex;
        frame_processor_callback_ptr _callback;
        synthetic_source _source_wrapper;
    };

    class LRS_EXTENSION_API generic_processing_block : public processing_block
    {
    public:
        generic_processing_block(const char* name);
        virtual ~generic_processing_block() { _source.flush(); }

    protected:
        virtual rs2::frame prepare_output(const rs2::frame_source& source, rs2::frame input, std::vector<rs2::frame> results);

        virtual bool should_process(const rs2::frame& frame) = 0;
        virtual rs2::frame process_frame(const rs2::frame_source& source, const rs2::frame& f) = 0;
    };

    struct stream_filter
    {
        rs2_stream stream;
        rs2_format format;
        int index;

        stream_filter() : stream(RS2_STREAM_ANY), format(RS2_FORMAT_ANY), index(-1) {}
        stream_filter(rs2_stream s, rs2_format f, int i) : stream(s), format(f), index(i) {}

        bool match(const rs2::frame& frame)
        {
            stream_filter filter(frame.get_profile().stream_type(), frame.get_profile().format(), frame.get_profile().stream_index());
            return match(filter);
        }

        bool match(const stream_filter& other)
        {
            if (stream != RS2_STREAM_ANY && stream != other.stream)
                return false;
            if (format != RS2_FORMAT_ANY && format != other.format)
                return false;
            if (index != -1 && index != other.index)
                return false;
            return true;
        }

        bool operator==(const stream_filter& other)
        {
            if (stream != other.stream)
                return false;
            if (format != other.format)
                return false;
            if (index != other.index)
                return false;
            return true;
        }

        bool operator!=(const stream_filter& other)
        {
            return !(*this == other);
        }

        void operator=(const stream_filter& other)
        {
            stream = other.stream;
            format = other.format;
            index = other.index;
        }
    };

    class LRS_EXTENSION_API stream_filter_processing_block : public generic_processing_block
    {
    public:
        stream_filter_processing_block(const char* name);
        virtual ~stream_filter_processing_block() { _source.flush(); }

    protected:
        stream_filter _stream_filter;

        bool should_process(const rs2::frame& frame) override;
    };

    class functional_processing_block : public stream_filter_processing_block
    {
    public:
        functional_processing_block(const char* name, rs2_format target_format, rs2_stream target_stream = RS2_STREAM_ANY, rs2_extension extension_type = RS2_EXTENSION_VIDEO_FRAME);

    protected:
        virtual void init(const rs2::frame* f);

        template<typename F>
        rs2::frame pre_process_frame(const rs2::frame_source& source, const rs2::frame& f, F process)
        {
            auto&& ret = prepare_frame(source, f);
            int width = 0;
            int height = 0;
            auto vf = ret.as<rs2::video_frame>();
            if (vf)
            {
                width = vf.get_width();
                height = vf.get_height();
            }
            byte* planes[1];
            planes[0] = (byte*)ret.get_data();

            process(_target_format, _target_stream, planes, (const byte*)f.get_data(), width, height, height * width * _target_bpp);

            return ret;
        };
        virtual rs2::frame prepare_frame(const rs2::frame_source& source, const rs2::frame& f);

        rs2::stream_profile _target_stream_profile;
        rs2::stream_profile _source_stream_profile;
        rs2_format _target_format;
        rs2_stream _target_stream;
        rs2_extension _extension_type;
        int _target_bpp = 0;
    };

    // handles interleaved frames with a defined function
    class interleaved_functional_processing_block : public processing_block
    {
    public:
        interleaved_functional_processing_block(const char* name,
            rs2_format source_format,
            rs2_format target_format,
            rs2_stream target_stream = RS2_STREAM_ANY,
            rs2_extension extension_type = RS2_EXTENSION_VIDEO_FRAME,
            int left_idx = 1,
            int right_idx = 2);

    protected:
        template<typename F>
        void configure_processing_callback(F process)
        {
            // define and set the frame processing callback
            auto process_callback = [&, process](frame_holder frame, synthetic_source_interface* source)
            {
                auto profile = As<video_stream_profile, stream_profile_interface>(frame.frame->get_stream());
                auto w = profile->get_width();
                auto h = profile->get_height();

                if (profile.get() != _source_stream_profile.get())
                {
                    _source_stream_profile = profile;
                    _target_stream_profile_right = profile->clone();
                    _target_stream_profile_left = profile->clone();

                    _target_bpp = get_image_bpp(_target_format) / 8;

                    _target_stream_profile_left->set_format(_target_format);
                    _target_stream_profile_right->set_format(_target_format);
                    _target_stream_profile_left->set_stream_type(profile->get_stream_type());
                    _target_stream_profile_right->set_stream_type(profile->get_stream_type());
                    _target_stream_profile_left->set_stream_index(_left_target_profile_idx);
                    _target_stream_profile_left->set_unique_id(_left_target_profile_idx);
                    _target_stream_profile_right->set_stream_index(_right_target_profile_idx);
                    _target_stream_profile_right->set_unique_id(_right_target_profile_idx);
                }

                // passthrough the frame if we don't need to process it.
                auto format = profile->get_format();
                if (format != _source_format)
                {
                    source->frame_ready(std::move(frame));
                    return;
                }

                frame_holder lf, rf;

                lf = source->allocate_video_frame(_target_stream_profile_left, frame, _target_bpp,
                    w, h, w * _target_bpp, _extension_type);
                rf = source->allocate_video_frame(_target_stream_profile_right, frame, _target_bpp,
                    w, h, w * _target_bpp, _extension_type);

                // process the frame
                byte* planes[2];
                planes[0] = (byte*)lf.frame->get_frame_data();
                planes[1] = (byte*)rf.frame->get_frame_data();
                process(_target_format, _target_stream, planes, (const byte*)frame->get_frame_data(), w, h, 0);

                source->frame_ready(std::move(lf));
                source->frame_ready(std::move(rf));
            };

            set_processing_callback(std::shared_ptr<rs2_frame_processor_callback>(
                new internal_frame_processor_callback<decltype(process_callback)>(process_callback)));
        };

        std::shared_ptr<stream_profile_interface> _source_stream_profile;
        std::shared_ptr<stream_profile_interface> _target_stream_profile_left;
        std::shared_ptr<stream_profile_interface> _target_stream_profile_right;
        rs2_format _source_format;
        rs2_format _target_format;
        rs2_stream _target_stream;
        rs2_extension _extension_type;
        int _target_bpp = 0;
        int _left_target_profile_idx = 1;
        int _right_target_profile_idx = 2;
    };

    class depth_processing_block : public stream_filter_processing_block
    {
    public:
        depth_processing_block(const char* name) : stream_filter_processing_block(name) {}

        virtual ~depth_processing_block() { _source.flush(); }

    protected:
        bool should_process(const rs2::frame& frame) override;
    };

    class LRS_EXTENSION_API composite_processing_block : public processing_block
    {
    public:
        class bypass_option : public option
        {
        public:
            bypass_option(composite_processing_block* parent, rs2_option opt)
                : _parent(parent), _opt(opt) {}

            void set(float value) override {
                // While query and other read operations
                // will only read from the currently selected
                // block, setting an option will propogate
                // to all blocks in the group
                for (int i = 0; i < _parent->_processing_blocks.size(); i++)
                {
                    if (_parent->_processing_blocks[i]->supports_option(_opt))
                    {
                        _parent->_processing_blocks[i]->get_option(_opt).set(value);
                    }
                }
            }
            float query() const override { return get().query(); }
            option_range get_range() const override { return get().get_range(); }
            bool is_enabled() const override { return get().is_enabled(); }
            bool is_read_only() const override { return get().is_read_only(); }
            const char* get_description() const override { return get().get_description(); }
            const char* get_value_description(float v) const override { return get().get_value_description(v); }
            void enable_recording(std::function<void(const option &)> record_action) override {}

            option& get() { return _parent->get(_opt).get_option(_opt); }
            const option& get() const { return _parent->get(_opt).get_option(_opt); }
        private:
            composite_processing_block* _parent;
            rs2_option _opt;
        };

        composite_processing_block();
        composite_processing_block(const char* name);
        virtual ~composite_processing_block() { _source.flush(); };

        processing_block& get(rs2_option option);
        void add(std::shared_ptr<processing_block> block);
        void set_output_callback(frame_callback_ptr callback) override;
        void invoke(frame_holder frames) override;

    protected:
        std::vector<std::shared_ptr<processing_block>> _processing_blocks;
    };
}

// API structures
struct rs2_options
{
    rs2_options(librealsense::options_interface* options) : options(options) { }

    librealsense::options_interface* options;

    virtual ~rs2_options() = default;
};

struct rs2_options_list
{
    std::vector<rs2_option> list;
};

struct rs2_processing_block : public rs2_options
{
    rs2_processing_block(std::shared_ptr<librealsense::processing_block_interface> block)
        : rs2_options((librealsense::options_interface*)block.get()),
        block(block) { }

    std::shared_ptr<librealsense::processing_block_interface> block;

    rs2_processing_block& operator=(const rs2_processing_block&) = delete;
    rs2_processing_block(const rs2_processing_block&) = delete;
};
