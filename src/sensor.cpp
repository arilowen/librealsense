// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.
#include <array>
#include <set>
#include <unordered_set>

#include "source.h"
#include "proc/synthetic-stream.h"
#include <iomanip>

#include "device.h"
#include "stream.h"
#include "sensor.h"
#include "proc/decimation-filter.h"
#include "global_timestamp_reader.h"
#include "metadata.h"

#include "../common/tiny-profiler.h"

namespace librealsense
{
    sensor_base::sensor_base(std::string name, device* dev,
        recommended_proccesing_blocks_interface* owner)
        : recommended_proccesing_blocks_base(owner),
        _is_streaming(false),
          _is_opened(false),
          _notifications_processor(std::shared_ptr<notifications_processor>(new notifications_processor())),
          _on_before_frame_callback(nullptr),
          _metadata_parsers(std::make_shared<metadata_parser_map>()),
          _on_open(nullptr),
          _owner(dev),
          _profiles([this]() {
                auto profiles = this->init_stream_profiles();
                _owner->tag_profiles(profiles);
                return profiles;
          })
    {
        register_option(RS2_OPTION_FRAMES_QUEUE_SIZE, _source.get_published_size_option());

        register_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL, std::make_shared<librealsense::md_time_of_arrival_parser>());

        register_info(RS2_CAMERA_INFO_NAME, name);
    }

    const std::string& sensor_base::get_info(rs2_camera_info info) const
    {
        if (info_container::supports_info(info)) return info_container::get_info(info);
        else return _owner->get_info(info);
    }

    bool sensor_base::supports_info(rs2_camera_info info) const
    {
        return info_container::supports_info(info) || _owner->supports_info(info);
    }

    stream_profiles sensor_base::get_active_streams() const
    {
        return _active_profiles;
    }

    void sensor_base::register_notifications_callback(notifications_callback_ptr callback)
    {
        if (supports_option(RS2_OPTION_ERROR_POLLING_ENABLED))
        {
            auto& opt = get_option(RS2_OPTION_ERROR_POLLING_ENABLED);
            opt.set(1.0f);
        }
        _notifications_processor->set_callback(std::move(callback));
    }

    notifications_callback_ptr sensor_base::get_notifications_callback() const
    {
        return _notifications_processor->get_callback();
    }

    int sensor_base::register_before_streaming_changes_callback(std::function<void(bool)> callback)
    {
        int token = (on_before_streaming_changes += callback);
        LOG_DEBUG("Registered token #" << token << " to \"on_before_streaming_changes\"");
        return token;
    }

    void sensor_base::unregister_before_start_callback(int token)
    {
        bool successful_unregister = on_before_streaming_changes -= token;
        if (!successful_unregister)
        {
            LOG_WARNING("Failed to unregister token #" << token << " from \"on_before_streaming_changes\"");
        }
    }

    frame_callback_ptr sensor_base::get_frames_callback() const
    {
        return _source.get_callback();
    }
    void sensor_base::set_frames_callback(frame_callback_ptr callback)
    {
        return _source.set_callback(callback);
    }
    std::shared_ptr<notifications_processor> sensor_base::get_notifications_processor()
    {
        return _notifications_processor;
    }

    void sensor_base::raise_on_before_streaming_changes(bool streaming)
    {
        on_before_streaming_changes(streaming);
    }
    void sensor_base::set_active_streams(const stream_profiles& requests)
    {
        _active_profiles = requests;
    }

    bool sensor_base::try_get_pf(const platform::stream_profile& p, native_pixel_format& result) const
    {
        auto it = std::find_if(begin(_pixel_formats), end(_pixel_formats),
            [&p](const native_pixel_format& pf)
        {
            return pf.fourcc == p.format;
        });
        if (it != end(_pixel_formats))
        {
            result = *it;
            return true;
        }
        return false;
    }

    void sensor_base::assign_stream(const std::shared_ptr<stream_interface>& stream, std::shared_ptr<stream_profile_interface> target) const
    {
        environment::get_instance().get_extrinsics_graph().register_same_extrinsics(*stream, *target);
        auto uid = stream->get_unique_id();
        target->set_unique_id(uid);
    }

    stream_profiles sensor_base::resolve_requests(stream_profiles requests)
    {
        // translate requests formats into fourcc formats
        stream_profiles resolved_requests;
        auto&& const profiles = get_stream_profiles();

        for (auto req : requests)
        {
            for (auto&& const entry : _fourcc_to_rs2_format)
            {
                auto fourcc_format = entry.first;
                auto rs2_format = entry.second;
                if (rs2_format == req->get_format())
                {
                    //// TODO - think to change to O(n^2) instead of O(n^3)
                    //auto profile_it = std::find_if(profiles.begin(), profiles.end(), [&fourcc_format, &resolved_req](std::shared_ptr<stream_profile_interface> sp)
                    //{
                    //    auto profile = std::dynamic_pointer_cast<stream_profile_base>(sp);
                    //    if (fourcc_format == profile->get_backend_profile().format)
                    //    {
                    //        resolved_req.push_back(profile);
                    //        return true;
                    //    }
                    //    return false;
                    //});

                    //if (profile_it == profiles.end())
                    //    throw invalid_value_exception("Request format is not supported by the device!");

                    //// TODO - Ariel - maybe convert given requests into resolved ones
                    //auto base_req = std::dynamic_pointer_cast<video_stream_profile>(req);
                    //auto resolved_req = std::make_shared<video_stream_profile>(base_req->get_backend_profile());
                    //resolved_req->set_format(advanced_to_backend_format(rs2_format));
                    //resolved_req->set_stream_type(base_req->get_stream_type());
                    //resolved_requests.push_back(resolved_req);
                    //req->set_format(advanced_to_backend_format(rs2_format));
                    //resolved_requests.push_back(req);
                    resolved_requests.push_back(req);

                    //auto profile = dynamic_cast<backend_stream_profile*>((*it).get())->get_backend_profile();
                    //resolved_req.push_back(*profile_it);
                    // TODO - handle request errors 
                    //requests.erase(req);
                }
            }
        }

        return resolved_requests;
    }

    std::vector<request_mapping> sensor_base::resolve_requestss(stream_profiles requests)
    {
        // per requested profile, find all 4ccs that support that request.
        std::map<int, std::set<uint32_t>> legal_fourccs;
        auto profiles = get_stream_profiles();
        for (auto&& r : requests)
        {
            auto sp = to_profile(r.get());
            for (auto&& mode : profiles)
            {
                if (auto backend_profile = dynamic_cast<backend_stream_profile*>(mode.get()))
                {
                    auto m = to_profile(mode.get());

                    if (m.fps == sp.fps && m.height == sp.height && m.width == sp.width)
                        legal_fourccs[sp.index].insert(backend_profile->get_backend_profile().format); // TODO: Stread ID???
                }
            }
        }

        //if you want more efficient data structure use std::unordered_set
        //with well-defined hash function
        std::set<request_mapping> results;
        while (!requests.empty() && !_pixel_formats.empty())
        {
            auto max = 0;
            size_t best_size = 0;
            auto best_pf = &_pixel_formats.front();
            auto best_unpacker = &_pixel_formats.front().unpackers.front();
            platform::stream_profile uvc_profile{};
            for (auto&& pf : _pixel_formats)
            {
                // Speeds up algorithm by skipping obviously useless 4ccs
                // if (std::none_of(begin(legal_fourccs), end(legal_fourccs), [&](const uint32_t fourcc) {return fourcc == pf.fourcc; })) continue;

                for (auto&& unpacker : pf.unpackers)
                {
                    auto count = static_cast<int>(std::count_if(begin(requests), end(requests),
                        [&pf, &legal_fourccs, &unpacker, this](const std::shared_ptr<stream_profile_interface>& r)
                    {
                        // only count if the 4cc can be unpacked into the relevant stream/format
                        // and also, the pixel format can be streamed in the requested dimensions/fps.
                        return unpacker.satisfies(to_profile(r.get()), pf.fourcc, _uvc_profiles) && legal_fourccs[r->get_stream_index()].count(pf.fourcc);
                    }));

                    // Here we check if the current pixel format / unpacker combination is better than the current best.
                    // We judge on two criteria. A: how many of the requested streams can we supply? B: how many total streams do we open?
                    // Optimally, we want to find a combination that supplies all the requested streams, and no additional streams.
                    if (
                        count > max                                 // If the current combination supplies more streams, it is better.
                        || (count == max                            // Alternatively, if it supplies the same number of streams,
                            && unpacker.outputs.size() < best_size) // but this combination opens fewer total streams, it is also better
                        )
                    {
                        max = count;
                        best_size = unpacker.outputs.size();
                        best_pf = &pf;
                        best_unpacker = &unpacker;
                    }
                }
            }

            if (max == 0) break;

            requests.erase(std::remove_if(begin(requests), end(requests),
                [best_unpacker, best_pf, &results, &legal_fourccs, this](const std::shared_ptr<stream_profile_interface>& r)
            {
                if (best_unpacker->satisfies(to_profile(r.get()), best_pf->fourcc, _uvc_profiles) && legal_fourccs[r->get_stream_index()].count(best_pf->fourcc))
                {
                    auto request = dynamic_cast<const video_stream_profile*>(r.get());

                    request_mapping mapping;
                    mapping.unpacker = best_unpacker;
                    mapping.pf = best_pf;
                    auto uvc_profile = best_unpacker->get_uvc_profile(to_profile(r.get()), best_pf->fourcc, _uvc_profiles);
                    if (!request) {
                        mapping.profile = { 0, 0, r->get_framerate(), best_pf->fourcc };
                    }
                    else
                    {
                        mapping.profile = { uvc_profile.width, uvc_profile.height, uvc_profile.fps, best_pf->fourcc };
                    }

                    results.insert(mapping);

                    auto it = results.find(mapping);
                    if (it != results.end())
                    {
                        it->original_requests.push_back(map_requests(r));
                    }

                    return true;
                }
                return false;
            }), end(requests));
        }

        if (requests.empty()) return{ begin(results), end(results) };

        throw invalid_value_exception("Subdevice unable to satisfy stream requests!");
    }

    std::shared_ptr<stream_profile_interface> sensor_base::map_requests(std::shared_ptr<stream_profile_interface> request)
    {
        stream_profiles results;
        auto profiles = get_stream_profiles();

        auto it = std::find_if(profiles.begin(), profiles.end(), [&](std::shared_ptr<stream_profile_interface> p) {
            return to_profile(p.get()) == to_profile(request.get());
        });

        if (it == profiles.end())
            throw invalid_value_exception("Subdevice could not map requests!");

        return *it;
    }

    uvc_sensor::~uvc_sensor()
    {
        try
        {
            if (_is_streaming)
                uvc_sensor::stop();

            if (_is_opened)
                uvc_sensor::close();
        }
        catch(...)
        {
            LOG_ERROR("An error has occurred while stop_streaming()!");
        }
    }

    stream_profiles sensor_base::get_stream_profiles(int tag) const
    {
        if (tag == profile_tag::PROFILE_TAG_ANY)
            return *_profiles;

        stream_profiles results;
        for (auto p : *_profiles)
        {
            auto curr_tag = p->get_tag();
            if (curr_tag & tag)
                results.push_back(p);
        }

        return results;
    }

    processing_blocks get_color_recommended_proccesing_blocks()
    {
        processing_blocks res;
        auto dec = std::make_shared<decimation_filter>();
        if (!dec->supports_option(RS2_OPTION_STREAM_FILTER))
            return res;
        dec->get_option(RS2_OPTION_STREAM_FILTER).set(RS2_STREAM_COLOR);
        dec->get_option(RS2_OPTION_STREAM_FORMAT_FILTER).set(RS2_FORMAT_ANY);
        res.push_back(dec);
        return res;
    }

    processing_blocks get_depth_recommended_proccesing_blocks()
    {
        processing_blocks res;
        auto dec = std::make_shared<decimation_filter>();
        if (dec->supports_option(RS2_OPTION_STREAM_FILTER))
        {
            dec->get_option(RS2_OPTION_STREAM_FILTER).set(RS2_STREAM_DEPTH);
            dec->get_option(RS2_OPTION_STREAM_FORMAT_FILTER).set(RS2_FORMAT_Z16);
            res.push_back(dec);
        }
        return res;
    }

    stream_profiles uvc_sensor::init_stream_profiles()
    {
        std::unordered_set<std::shared_ptr<video_stream_profile>> profiles;
        power on(std::dynamic_pointer_cast<uvc_sensor>(shared_from_this()));

        if (_uvc_profiles.empty()) {}
        _uvc_profiles = _device->get_profiles();

        for (auto&& p : _uvc_profiles)
        {
            auto fourcc_fmt = fourcc_to_rs2_format(p.format);
            if (fourcc_fmt == RS2_FORMAT_ANY)
                continue;

            auto profile = std::make_shared<video_stream_profile>(p);
            profile->set_dims(p.width, p.height);
            profile->set_stream_type(fourcc_to_rs2_stream(p.format));
            profile->set_stream_index(0);
            profile->set_format(fourcc_fmt);
            profile->set_framerate(p.fps);
            profiles.insert(profile);
        }

        stream_profiles result{profiles.begin(), profiles.end()};
        return result;
    }

    // TODO - delete
    //stream_profiles uvc_sensor::init_stream_profiles()
    //{
    //    std::unordered_set<std::shared_ptr<video_stream_profile>> results;
    //    std::set<uint32_t> unregistered_formats;
    //    std::set<uint32_t> supported_formats;

    //    power on(std::dynamic_pointer_cast<uvc_sensor>(shared_from_this()));
    //    if (_uvc_profiles.empty()){}
    //        _uvc_profiles = _device->get_profiles();

    //    for (auto&& p : _uvc_profiles)
    //    {
    //        supported_formats.insert(p.format);
    //        native_pixel_format pf{};
    //        if (try_get_pf(p, pf))
    //        {
    //            for (auto&& unpacker : pf.unpackers)
    //            {
    //                for (auto&& output : unpacker.outputs)
    //                {
    //                    auto profile = std::make_shared<video_stream_profile>(p);
    //                    auto res = output.stream_resolution({ p.width, p.height });
    //                    profile->set_dims(res.width, res.height);
    //                    profile->set_stream_type(output.stream_desc.type);
    //                    profile->set_stream_index(output.stream_desc.index);
    //                    profile->set_format(output.format);
    //                    profile->set_framerate(p.fps);
    //                    results.insert(profile);
    //                }
    //            }
    //        }
    //        else
    //        {
    //            unregistered_formats.insert(p.format);
    //        }
    //    }

    //    if (unregistered_formats.size())
    //    {
    //        std::stringstream ss;
    //        ss << "Unregistered Media formats : [ ";
    //        for (auto& elem : unregistered_formats)
    //        {
    //            uint32_t device_fourcc = reinterpret_cast<const big_endian<uint32_t>&>(elem);
    //            char fourcc[sizeof(device_fourcc) + 1];
    //            librealsense::copy(fourcc, &device_fourcc, sizeof(device_fourcc));
    //            fourcc[sizeof(device_fourcc)] = 0;
    //            ss << fourcc << " ";
    //        }

    //        ss << "]; Supported: [ ";
    //        for (auto& elem : supported_formats)
    //        {
    //            uint32_t device_fourcc = reinterpret_cast<const big_endian<uint32_t>&>(elem);
    //            char fourcc[sizeof(device_fourcc) + 1];
    //            librealsense::copy(fourcc, &device_fourcc, sizeof(device_fourcc));
    //            fourcc[sizeof(device_fourcc)] = 0;
    //            ss << fourcc << " ";
    //        }
    //        ss << "]";
    //        LOG_INFO(ss.str());
    //    }

    //    // Sort the results to make sure that the user will receive predictable deterministic output from the API
    //    stream_profiles res{ begin(results), end(results) };
    //    std::sort(res.begin(), res.end(), [](const std::shared_ptr<stream_profile_interface>& ap,
    //                                         const std::shared_ptr<stream_profile_interface>& bp)
    //    {
    //        auto a = to_profile(ap.get());
    //        auto b = to_profile(bp.get());

    //        // stream == RS2_STREAM_COLOR && format == RS2_FORMAT_RGB8 element works around the fact that Y16 gets priority over RGB8 when both
    //        // are available for pipeline stream resolution
    //        auto at = std::make_tuple(a.stream, a.width, a.height, a.fps, a.stream == RS2_STREAM_COLOR && a.format == RS2_FORMAT_RGB8, a.format);
    //        auto bt = std::make_tuple(b.stream, b.width, b.height, b.fps, b.stream == RS2_STREAM_COLOR && b.format == RS2_FORMAT_RGB8, b.format);

    //        return at > bt;
    //    });

    //    return res;
    //}

    rs2_extension uvc_sensor::stream_to_frame_types(rs2_stream stream) const
    {
        // TODO: explicitly return video_frame for relevant streams and default to an error?
        switch (stream)
        {
        case RS2_STREAM_DEPTH:  return RS2_EXTENSION_DEPTH_FRAME;
        default:                return RS2_EXTENSION_VIDEO_FRAME;
        }
    }

    device_interface& sensor_base::get_device()
    {
        return *_owner;
    }

    void sensor_base::register_pixel_format(native_pixel_format pf)
    {
        if (_pixel_formats.end() == std::find_if(_pixel_formats.begin(), _pixel_formats.end(),
            [&pf](const native_pixel_format& cur) { return cur.fourcc == pf.fourcc; }))
            _pixel_formats.push_back(pf);
        else
            throw invalid_value_exception(to_string()
                << "Pixel format " << std::hex << std::setw(8) << std::setfill('0') << pf.fourcc
                << " has been already registered with the sensor " << get_info(RS2_CAMERA_INFO_NAME));
    }

    void sensor_base::remove_pixel_format(native_pixel_format pf)
    {
        auto it = std::find_if(_pixel_formats.begin(), _pixel_formats.end(), [&pf](const native_pixel_format& cur) { return cur.fourcc == pf.fourcc; });
        if (it != _pixel_formats.end())
            _pixel_formats.erase(it);
    }

    void uvc_sensor::open(const stream_profiles& requests)
    {
        std::lock_guard<std::mutex> lock(_configure_lock);
        if (_is_streaming)
            throw wrong_api_call_sequence_exception("open(...) failed. UVC device is streaming!");
        else if (_is_opened)
            throw wrong_api_call_sequence_exception("open(...) failed. UVC device is already opened!");

        auto on = std::unique_ptr<power>(new power(std::dynamic_pointer_cast<uvc_sensor>(shared_from_this())));

        _source.init(_metadata_parsers);
        //_source.set_sensor(this->shared_from_this());
        _source.set_sensor(_sensor_owner);
        auto req_profiles = resolve_requests(requests);

        auto timestamp_reader = _timestamp_reader.get();

        std::vector<platform::stream_profile> commited;

        for (auto&& req_profile : req_profiles)
        {
            auto req_profile_base = std::dynamic_pointer_cast<stream_profile_base>(req_profile);
            try
            {
                unsigned long long last_frame_number = 0;
                rs2_time_t last_timestamp = 0;
                _device->probe_and_commit(req_profile_base->get_backend_profile(),
                    [this, timestamp_reader, req_profile_base, last_frame_number, last_timestamp](platform::stream_profile p, platform::frame_object f, std::function<void()> continuation) mutable
                {
                    auto system_time = environment::get_instance().get_time_service()->get_time();
                    if (!this->is_streaming())
                    {
                        LOG_WARNING("Frame received with streaming inactive,"
                            << librealsense::get_string(req_profile_base->get_stream_type())
                            << req_profile_base->get_stream_index()
                            << ", Arrived," << std::fixed << f.backend_time << " " << system_time);
                        return;
                    }

                    frame_continuation release_and_enqueue(continuation, f.pixels);
                    

                    // Ignore any frames which appear corrupted or invalid
                    // Determine the timestamp for this frame

                    std::shared_ptr<frame> frame(new frame);
                    double timestamp = 0;
                    rs2_timestamp_domain timestamp_domain;
                    unsigned long long frame_counter = 0;

                    frame_additional_data additional_data(timestamp,
                        frame_counter,
                        system_time,
                        static_cast<uint8_t>(f.metadata_size),
                        (const uint8_t*)f.metadata,
                        f.backend_time,
                        last_timestamp,
                        last_frame_number,
                        false);
                    frame->additional_data = additional_data;
                    byte* pix = (byte*)f.pixels;
                    std::vector<byte> pixels (pix, pix + f.frame_size);
                    frame->data = pixels;
                    frame->set_stream(req_profile_base);
                        
                    timestamp = timestamp_reader->get_frame_timestamp(frame);
                    additional_data.timestamp = timestamp;
                        
                    timestamp_domain = timestamp_reader->get_frame_timestamp_domain(frame);
                    additional_data.last_frame_number =
                    frame_counter = timestamp_reader->get_frame_counter(frame);
                    additional_data.frame_number = frame_counter;

                    auto requires_processing = true;//mode.requires_processing(); // TODO - Ariel add option

                  /*  std::vector<byte *> dest;*/
                    //std::vector<frame_holder> refs;

                    //auto&& unpacker = *mode.unpacker;
                    //for (auto&& output : unpacker.outputs)
                    //{
                        LOG_DEBUG("FrameAccepted," << librealsense::get_string(req_profile_base->get_stream_type())
                            << ",Counter," << std::dec << frame_counter
                            << ",Index," << req_profile_base->get_stream_index()
                            << ",BackEndTS," << std::fixed << f.backend_time
                            << ",SystemTime," << std::fixed << system_time
                            << " ,diff_ts[Sys-BE]," << system_time - f.backend_time
                            << ",TS," << std::fixed << timestamp << ",TS_Domain," << rs2_timestamp_domain_to_string(timestamp_domain)
                            << ",last_frame_number," << last_frame_number << ",last_timestamp," << last_timestamp);

                        /*std::shared_ptr<stream_profile_interface> request = nullptr;
                        for (auto&& original_prof : mode.original_requests)
                        {
                            if (original_prof->get_format() == output.format &&
                                original_prof->get_stream_type() == output.stream_desc.type &&
                                original_prof->get_stream_index() == output.stream_desc.index)
                            {
                                request = original_prof;
                            }
                        }*/

                        auto bpp = get_image_bpp(req_profile_base->get_format());
                        

                        last_frame_number = frame_counter;
                        last_timestamp = timestamp;

                        auto backend_sp = req_profile_base->get_backend_profile();
                        auto width = backend_sp.width;
                        auto height = backend_sp.height;

                        frame_holder fh = _source.alloc_frame(stream_to_frame_types(req_profile_base->get_stream_type()), width * height * bpp / 8, additional_data, requires_processing);
                        if (fh.frame)
                        {
                            memcpy((void*)fh->get_frame_data(), frame->data.data(), sizeof(byte)*frame->data.size());
                            auto video = (video_frame*)fh.frame;
                            video->assign(width, height, width * bpp / 8, bpp);
                            video->set_timestamp_domain(timestamp_domain);
                            //dest.push_back(const_cast<byte*>(video->get_frame_data()));
                            fh->set_stream(req_profile_base);
                            //refs.push_back(std::move(fh));
                        }
                        else
                        {
                            LOG_INFO("Dropped frame. alloc_frame(...) returned nullptr");
                            return;
                        }

                        // Obtain buffers for unpacking the frame
                        //dest.push_back(archive->alloc_frame(output.first, additional_data, requires_processing));
                    //}

                    // Unpack the frame
                    //if (requires_processing && (dest.size() > 0))
                    //{
                        //unpacker.unpack(dest.data(), reinterpret_cast<const byte *>(f.pixels), mode.profile.width, mode.profile.height);
                    //}

                    // If any frame callbacks were specified, dispatch them now
                    /*for (auto&& pref : refs)
                    {
                        scoped_timer t("for loop scope timer");*/
                        if (!requires_processing)
                        {
                            fh->attach_continuation(std::move(release_and_enqueue));
                        }

                        //if (_on_before_frame_callback) // TODO - Ariel DELETE?
                        //{
                        //    auto callback = _source.begin_callback();
                        //    auto stream_type = fh->get_stream()->get_stream_type();
                        //    _on_before_frame_callback(stream_type, fh, std::move(callback));
                        //}

                        if (fh->get_stream().get())
                        {
                            _source.invoke_callback(std::move(fh));
                        }
                    //}
                });
            }
            catch (...)
            {
                for (auto&& commited_profile : commited)
                {
                    _device->close(commited_profile);
                }
                throw;
            }
            commited.push_back(req_profile_base->get_backend_profile());
        }

        _internal_config = commited;

        if (_on_open)
            _on_open(_internal_config);

        _power = move(on);
        _is_opened = true;

        try {
            _device->stream_on([&](const notification& n)
            {
                _notifications_processor->raise_notification(n);
            });
        }
        catch (...)
        {
            for (auto& profile : _internal_config)
            {
                try {
                    _device->close(profile);
                }
                catch (...) {}
            }
            reset_streaming();
            _power.reset();
            _is_opened = false;
            throw;
        }
        if (Is<librealsense::global_time_interface>(_owner))
        {
            As<librealsense::global_time_interface>(_owner)->enable_time_diff_keeper(true);
        }
        set_active_streams(requests);
    }

    void uvc_sensor::opens(const stream_profiles& requests)
    {
    //    std::lock_guard<std::mutex> lock(_configure_lock);
    //    if (_is_streaming)
    //        throw wrong_api_call_sequence_exception("open(...) failed. UVC device is streaming!");
    //    else if (_is_opened)
    //        throw wrong_api_call_sequence_exception("open(...) failed. UVC device is already opened!");

    //    auto on = std::unique_ptr<power>(new power(std::dynamic_pointer_cast<uvc_sensor>(shared_from_this())));

    //    _source.init(_metadata_parsers);
    //    _source.set_sensor(this->shared_from_this());
    //    auto mapping = resolve_requestss(requests);

    //    auto timestamp_reader = _timestamp_reader.get();

    //    std::vector<platform::stream_profile> commited;

    //    for (auto&& mode : mapping)
    //    {
    //        try
    //        {
    //            unsigned long long last_frame_number = 0;
    //            rs2_time_t last_timestamp = 0;
    //            _device->probe_and_commit(mode.profile,
    //            [this, mode, timestamp_reader, requests, last_frame_number, last_timestamp](platform::stream_profile p, platform::frame_object f, std::function<void()> continuation) mutable
    //            {
    //                auto system_time = environment::get_instance().get_time_service()->get_time();
    //                if (!this->is_streaming())
    //                {
    //                    LOG_WARNING("Frame received with streaming inactive,"
    //                        << librealsense::get_string(mode.unpacker->outputs.front().stream_desc.type)
    //                        << mode.unpacker->outputs.front().stream_desc.index
    //                            << ", Arrived," << std::fixed << f.backend_time << " " << system_time);
    //                    return;
    //                }

    //                frame_continuation release_and_enqueue(continuation, f.pixels);

    //                // Ignore any frames which appear corrupted or invalid
    //                // Determine the timestamp for this frame
    //                auto timestamp = timestamp_reader->get_frame_timestamp(mode.profile, f);
    //                auto timestamp_domain = timestamp_reader->get_frame_timestamp_domain(mode.profile, f);
    //                auto frame_counter = timestamp_reader->get_frame_counter(mode.profile, f);

    //                auto requires_processing = mode.requires_processing();

    //                std::vector<byte *> dest;
    //                std::vector<frame_holder> refs;

    //                auto&& unpacker = *mode.unpacker;
    //                for (auto&& output : unpacker.outputs)
    //                {
    //                    LOG_DEBUG("FrameAccepted," << librealsense::get_string(output.stream_desc.type)
    //                        << ",Counter," << std::dec << frame_counter
    //                        << ",Index," << output.stream_desc.index
    //                        << ",BackEndTS," << std::fixed << f.backend_time
    //                        << ",SystemTime," << std::fixed << system_time
    //                        <<" ,diff_ts[Sys-BE],"<< system_time- f.backend_time
    //                        << ",TS," << std::fixed << timestamp << ",TS_Domain," << rs2_timestamp_domain_to_string(timestamp_domain)
    //                        <<",last_frame_number,"<< last_frame_number<<",last_timestamp,"<< last_timestamp);

    //                    std::shared_ptr<stream_profile_interface> request = nullptr;
    //                    for (auto&& original_prof : mode.original_requests)
    //                    {
    //                        if (original_prof->get_format() == output.format &&
    //                            original_prof->get_stream_type() == output.stream_desc.type &&
    //                            original_prof->get_stream_index() == output.stream_desc.index)
    //                        {
    //                            request = original_prof;
    //                        }
    //                    }

    //                    auto bpp = get_image_bpp(output.format);
    //                    frame_additional_data additional_data(timestamp,
    //                        frame_counter,
    //                        system_time,
    //                        static_cast<uint8_t>(f.metadata_size),
    //                        (const uint8_t*)f.metadata,
    //                        f.backend_time,
    //                        last_timestamp,
    //                        last_frame_number,
    //                        false);

    //                    last_frame_number = frame_counter;
    //                    last_timestamp = timestamp;

    //                    auto res = output.stream_resolution({ mode.profile.width, mode.profile.height });
    //                    auto width = res.width;
    //                    auto height = res.height;

    //                    frame_holder frame = _source.alloc_frame(stream_to_frame_types(output.stream_desc.type), width * height * bpp / 8, additional_data, requires_processing);
    //                    if (frame.frame)
    //                    {
    //                        auto video = (video_frame*)frame.frame;
    //                        video->assign(width, height, width * bpp / 8, bpp);
    //                        video->set_timestamp_domain(timestamp_domain);
    //                        dest.push_back(const_cast<byte*>(video->get_frame_data()));
    //                        frame->set_stream(request);
    //                        refs.push_back(std::move(frame));
    //                    }
    //                    else
    //                    {
    //                        LOG_INFO("Dropped frame. alloc_frame(...) returned nullptr");
    //                        return;
    //                    }

    //                    // Obtain buffers for unpacking the frame
    //                    //dest.push_back(archive->alloc_frame(output.first, additional_data, requires_processing));
    //                }

    //                // Unpack the frame
    //                if (requires_processing && (dest.size() > 0))
    //                {
    //                    unpacker.unpack(dest.data(), reinterpret_cast<const byte *>(f.pixels), mode.profile.width, mode.profile.height);
    //                }

    //                // If any frame callbacks were specified, dispatch them now
    //                for (auto&& pref : refs)
    //                {
    //                    if (!requires_processing)
    //                    {
    //                        pref->attach_continuation(std::move(release_and_enqueue));
    //                    }

    //                    if (_on_before_frame_callback)
    //                    {
    //                        auto callback = _source.begin_callback();
    //                        auto stream_type = pref->get_stream()->get_stream_type();
    //                        _on_before_frame_callback(stream_type, pref, std::move(callback));
    //                    }

    //                    if (pref->get_stream().get())
    //                        _source.invoke_callback(std::move(pref));
    //                }
    //            });
    //        }
    //        catch(...)
    //        {
    //            for (auto&& commited_profile : commited)
    //            {
    //                _device->close(commited_profile);
    //            }
    //            throw;
    //        }
    //        commited.push_back(mode.profile);
    //    }

    //    _internal_config = commited;

    //    if (_on_open)
    //        _on_open(_internal_config);

    //    _power = move(on);
    //    _is_opened = true;

    //    try {
    //        _device->stream_on([&](const notification& n)
    //        {
    //            _notifications_processor->raise_notification(n);
    //        });
    //    }
    //    catch (...)
    //    {
    //        for (auto& profile : _internal_config)
    //        {
    //            try {
    //                _device->close(profile);
    //            }
    //            catch (...) {}
    //        }
    //        reset_streaming();
    //        _power.reset();
    //        _is_opened = false;
    //        throw;
    //    }
    //    if (Is<librealsense::global_time_interface>(_owner))
    //    {
    //        As<librealsense::global_time_interface>(_owner)->enable_time_diff_keeper(true);
    //    }
    //    set_active_streams(requests);
    //}
    }

    void uvc_sensor::close()
    {
        std::lock_guard<std::mutex> lock(_configure_lock);
        if (_is_streaming)
            throw wrong_api_call_sequence_exception("close() failed. UVC device is streaming!");
        else if (!_is_opened)
            throw wrong_api_call_sequence_exception("close() failed. UVC device was not opened!");

        for (auto& profile : _internal_config)
        {
            try // Handle disconnect event
            {
                _device->close(profile);
            }
            catch (...) {}
        }
        reset_streaming();
        if (Is<librealsense::global_time_interface>(_owner))
        {
            As<librealsense::global_time_interface>(_owner)->enable_time_diff_keeper(false);
        }
        _power.reset();
        _is_opened = false;
        set_active_streams({});
    }

    void uvc_sensor::register_xu(platform::extension_unit xu)
    {
        _xus.push_back(std::move(xu));
    }

    void uvc_sensor::start(frame_callback_ptr callback)
    {
        std::lock_guard<std::mutex> lock(_configure_lock);
        if (_is_streaming)
            throw wrong_api_call_sequence_exception("start_streaming(...) failed. UVC device is already streaming!");
        else if(!_is_opened)
            throw wrong_api_call_sequence_exception("start_streaming(...) failed. UVC device was not opened!");

        _source.set_callback(callback);
        _is_streaming = true;
        raise_on_before_streaming_changes(true); //Required to be just before actual start allow recording to work
        _device->start_callbacks();
    }

    void uvc_sensor::stop()
    {
        std::lock_guard<std::mutex> lock(_configure_lock);
        if (!_is_streaming)
            throw wrong_api_call_sequence_exception("stop_streaming() failed. UVC device is not streaming!");

        _is_streaming = false;
        _device->stop_callbacks();
        raise_on_before_streaming_changes(false);
    }


    void uvc_sensor::reset_streaming()
    {
        _source.flush();
        _source.reset();
        _timestamp_reader->reset();
    }

    void uvc_sensor::acquire_power()
    {
        std::lock_guard<std::mutex> lock(_power_lock);
        if (_user_count.fetch_add(1) == 0)
        {
            _device->set_power_state(platform::D0);
            for (auto& xu : _xus) _device->init_xu(xu);
        }
    }

    void uvc_sensor::release_power()
    {
        std::lock_guard<std::mutex> lock(_power_lock);
        if (_user_count.fetch_add(-1) == 1)
        {
            _device->set_power_state(platform::D3);
        }
    }

    bool info_container::supports_info(rs2_camera_info info) const
    {
        auto it = _camera_info.find(info);
        return it != _camera_info.end();
    }

    void info_container::register_info(rs2_camera_info info, const std::string& val)
    {
        if (info_container::supports_info(info) && (info_container::get_info(info) != val)) // Append existing infos
        {
            _camera_info[info] += "\n" + std::move(val);
        }
        else
        {
            _camera_info[info] = std::move(val);
        }
    }

    void info_container::update_info(rs2_camera_info info, const std::string& val)
    {
        if (info_container::supports_info(info))
        {
            _camera_info[info] = std::move(val);
        }
    }
    const std::string& info_container::get_info(rs2_camera_info info) const
    {
        auto it = _camera_info.find(info);
        if (it == _camera_info.end())
            throw invalid_value_exception("Selected camera info is not supported for this camera!");

        return it->second;
    }
    void info_container::create_snapshot(std::shared_ptr<info_interface>& snapshot) const
    {
        snapshot = std::make_shared<info_container>(*this);
    }
    void info_container::enable_recording(std::function<void(const info_interface&)> record_action)
    {
       //info container is a read only class, nothing to record
    }

    void info_container::update(std::shared_ptr<extension_snapshot> ext)
    {
        if (auto info_api = As<info_interface>(ext))
        {
            for (int i = 0; i < RS2_CAMERA_INFO_COUNT; ++i)
            {
                rs2_camera_info info = static_cast<rs2_camera_info>(i);
                if (info_api->supports_info(info))
                {
                    register_info(info, info_api->get_info(info));
                }
            }
        }
    }

    void uvc_sensor::register_pu(rs2_option id)
    {
        register_option(id, std::make_shared<uvc_pu_option>(*this, id));
    }

    void uvc_sensor::try_register_pu(rs2_option id)
    {
        auto opt = std::make_shared<uvc_pu_option>(*this, id);
        try
        {
            auto range = opt->get_range();
            if (range.max <= range.min || range.step <= 0 || range.def < range.min || range.def > range.max) return;

            auto val = opt->query();
            if (val < range.min || val > range.max) return;
            opt->set(val);

            register_option(id, opt);
        }
        catch (...)
        {
            LOG_WARNING("Exception was thrown when inspecting " << this->get_info(RS2_CAMERA_INFO_NAME) << " property " << opt->get_description());
        }
    }

    void sensor_base::register_metadata(rs2_frame_metadata_value metadata, std::shared_ptr<md_attribute_parser_base> metadata_parser) const
    {
        if (_metadata_parsers.get()->end() != _metadata_parsers.get()->find(metadata))
            throw invalid_value_exception( to_string() << "Metadata attribute parser for " << rs2_frame_metadata_to_string(metadata)
                                           <<  " is already defined");

        _metadata_parsers.get()->insert(std::pair<rs2_frame_metadata_value, std::shared_ptr<md_attribute_parser_base>>(metadata, metadata_parser));
    }

    hid_sensor::hid_sensor(std::shared_ptr<platform::hid_device> hid_device, std::unique_ptr<frame_timestamp_reader> hid_iio_timestamp_reader,
        std::unique_ptr<frame_timestamp_reader> custom_hid_timestamp_reader,
        std::map<rs2_stream, std::map<unsigned, unsigned>> fps_and_sampling_frequency_per_rs2_stream,
        std::vector<std::pair<std::string, stream_profile>> sensor_name_and_hid_profiles,
        device* dev)
    : sensor_base("Motion Module", dev, (recommended_proccesing_blocks_interface*)this), _sensor_name_and_hid_profiles(sensor_name_and_hid_profiles),
      _fps_and_sampling_frequency_per_rs2_stream(fps_and_sampling_frequency_per_rs2_stream),
      _hid_device(hid_device),
      _is_configured_stream(RS2_STREAM_COUNT),
      _hid_iio_timestamp_reader(move(hid_iio_timestamp_reader)),
      _custom_hid_timestamp_reader(move(custom_hid_timestamp_reader))
    {
        register_metadata(RS2_FRAME_METADATA_BACKEND_TIMESTAMP, make_additional_data_parser(&frame_additional_data::backend_timestamp));

        std::map<std::string, uint32_t> frequency_per_sensor;
        for (auto& elem : sensor_name_and_hid_profiles)
            frequency_per_sensor.insert(make_pair(elem.first, elem.second.fps));

        std::vector<platform::hid_profile> profiles_vector;
        for (auto& elem : frequency_per_sensor)
            profiles_vector.push_back(platform::hid_profile{elem.first, elem.second});

        _hid_device->open(profiles_vector);
        for (auto& elem : _hid_device->get_sensors())
            _hid_sensors.push_back(elem);

        _hid_device->close();
    }

    hid_sensor::~hid_sensor()
    {
        try
        {
            if (_is_streaming)
                stop();

            if (_is_opened)
                close();
        }
        catch(...)
        {
            LOG_ERROR("An error has occurred while stop_streaming()!");
        }
    }

    stream_profiles hid_sensor::get_sensor_profiles(std::string sensor_name) const
    {
        stream_profiles profiles{};
        for (auto& elem : _sensor_name_and_hid_profiles)
        {
            if (!elem.first.compare(sensor_name))
            {
                auto p = elem.second;
                platform::stream_profile sp{ 1, 1, p.fps, stream_to_fourcc(p.stream) };
                auto profile = std::make_shared<motion_stream_profile>(sp);
                profile->set_stream_index(p.index);
                profile->set_stream_type(p.stream);
                profile->set_format(p.format);
                profile->set_framerate(p.fps);
                profiles.push_back(profile);
            }
        }

        return profiles;
    }

    void hid_sensor::open(const stream_profiles& requests)
    {
        std::lock_guard<std::mutex> lock(_configure_lock);
        if (_is_streaming)
            throw wrong_api_call_sequence_exception("open(...) failed. Hid device is streaming!");
        else if (_is_opened)
            throw wrong_api_call_sequence_exception("Hid device is already opened!");

        auto mapping = resolve_requestss(requests);
        for (auto& request : requests)
        {
            auto sensor_name = rs2_stream_to_sensor_name(request->get_stream_type());
            for (auto& map : mapping)
            {
                auto it = std::find_if(begin(map.unpacker->outputs), end(map.unpacker->outputs),
                                       [&](const stream_output& pair)
                {
                    return pair.stream_desc.type == request->get_stream_type() &&
                           pair.stream_desc.index == request->get_stream_index();
                });

                if (it != end(map.unpacker->outputs))
                {
                    _configured_profiles.insert(std::make_pair(sensor_name,
                                                               stream_profile{ request->get_stream_type(), request->get_stream_index(),
                                                                              0,
                                                                              0,
                                                                              fps_to_sampling_frequency(request->get_stream_type(), request->get_framerate()),
                                                                              request->get_format()}));
                    _is_configured_stream[request->get_stream_type()] = true;
                    _hid_mapping.insert(std::make_pair(sensor_name, map));
                }
            }
        }

        std::vector<platform::hid_profile> configured_hid_profiles;
        for (auto& elem : _configured_profiles)
        {
            configured_hid_profiles.push_back(platform::hid_profile{elem.first, elem.second.fps});
        }
        _hid_device->open(configured_hid_profiles);
        if (Is<librealsense::global_time_interface>(_owner))
        {
            As<librealsense::global_time_interface>(_owner)->enable_time_diff_keeper(true);
        }
        _is_opened = true;
        set_active_streams(requests);
    }

    void hid_sensor::close()
    {
        std::lock_guard<std::mutex> lock(_configure_lock);
        if (_is_streaming)
            throw wrong_api_call_sequence_exception("close() failed. Hid device is streaming!");
        else if (!_is_opened)
            throw wrong_api_call_sequence_exception("close() failed. Hid device was not opened!");

        _hid_device->close();
        _configured_profiles.clear();
        _is_configured_stream.clear();
        _is_configured_stream.resize(RS2_STREAM_COUNT);
        _hid_mapping.clear();
        _is_opened = false;
        if (Is<librealsense::global_time_interface>(_owner))
        {
            As<librealsense::global_time_interface>(_owner)->enable_time_diff_keeper(false);
        }
        set_active_streams({});
    }

    // TODO:
    static rs2_stream custom_gpio_to_stream_type(uint32_t custom_gpio)
    {
        if (custom_gpio < 4)
        {
            return static_cast<rs2_stream>(RS2_STREAM_GPIO);
        }

        LOG_ERROR("custom_gpio " << std::to_string(custom_gpio) << " is incorrect!");
        return RS2_STREAM_ANY;
    }

    void hid_sensor::start(frame_callback_ptr callback)
    {
        //std::lock_guard<std::mutex> lock(_configure_lock);
        //if (_is_streaming)
        //    throw wrong_api_call_sequence_exception("start_streaming(...) failed. Hid device is already streaming!");
        //else if(!_is_opened)
        //    throw wrong_api_call_sequence_exception("start_streaming(...) failed. Hid device was not opened!");

        //_source.set_callback(callback);

        //_source.init(_metadata_parsers);
        //_source.set_sensor(this->shared_from_this());
        //unsigned long long last_frame_number = 0;
        //rs2_time_t last_timestamp = 0;
        //raise_on_before_streaming_changes(true); //Required to be just before actual start allow recording to work
        //_hid_device->start_capture([this,last_frame_number,last_timestamp](const platform::sensor_data& sensor_data) mutable
        //{
        //    auto system_time = environment::get_instance().get_time_service()->get_time();
        //    auto timestamp_reader = _hid_iio_timestamp_reader.get();

        //    // TODO:
        //    static const std::string custom_sensor_name = "custom";
        //    auto sensor_name = sensor_data.sensor.name;
        //    bool is_custom_sensor = false;
        //    static const uint32_t custom_source_id_offset = 16;
        //    uint8_t custom_gpio = 0;
        //    auto custom_stream_type = RS2_STREAM_ANY;
        //    if (sensor_name == custom_sensor_name)
        //    {
        //        custom_gpio = *(reinterpret_cast<uint8_t*>((uint8_t*)(sensor_data.fo.pixels) + custom_source_id_offset));
        //        custom_stream_type = custom_gpio_to_stream_type(custom_gpio);

        //        if (!_is_configured_stream[custom_stream_type])
        //        {
        //            LOG_DEBUG("Unrequested " << rs2_stream_to_string(custom_stream_type) << " frame was dropped.");
        //            return;
        //        }

        //        is_custom_sensor = true;
        //        timestamp_reader = _custom_hid_timestamp_reader.get();
        //    }

        //    if (!this->is_streaming())
        //    {
        //        LOG_INFO("HID Frame received when Streaming is not active,"
        //                    << get_string(_configured_profiles[sensor_name].stream)
        //                    << ",Arrived," << std::fixed << system_time);
        //        return;
        //    }

        //    auto mode = _hid_mapping[sensor_name];
        //    auto request = *(mode.original_requests.begin());
        //    auto data_size = sensor_data.fo.frame_size;
        //    mode.profile.width = (uint32_t)data_size;
        //    mode.profile.height = 1;

        //    // Determine the timestamp for this HID frame
        //    auto timestamp = timestamp_reader->get_frame_timestamp(mode.profile, sensor_data.fo);
        //    auto frame_counter = timestamp_reader->get_frame_counter(mode.profile, sensor_data.fo);
        //    auto ts_domain = timestamp_reader->get_frame_timestamp_domain(mode.profile, sensor_data.fo);

        //    frame_additional_data additional_data(timestamp,
        //        frame_counter,
        //        system_time,
        //        static_cast<uint8_t>(sensor_data.fo.metadata_size),
        //        (const uint8_t*)sensor_data.fo.metadata,
        //        sensor_data.fo.backend_time,
        //        last_timestamp,
        //        last_frame_number,
        //        false);

        //    additional_data.timestamp_domain = ts_domain;
        //    additional_data.backend_timestamp = sensor_data.fo.backend_time;

        //    LOG_DEBUG("FrameAccepted," << get_string(request->get_stream_type())
        //            << ",Counter," << std::dec << frame_counter << ",Index,0"
        //            << ",BackEndTS," << std::fixed << sensor_data.fo.backend_time
        //            << ",SystemTime," << std::fixed << system_time
        //            <<" ,diff_ts[Sys-BE],"<< system_time- sensor_data.fo.backend_time
        //            << ",TS," << std::fixed << timestamp << ",TS_Domain," << rs2_timestamp_domain_to_string(ts_domain)
        //            <<",last_frame_number,"<< last_frame_number<<",last_timestamp,"<< last_timestamp);

        //    last_frame_number = frame_counter;
        //    last_timestamp = timestamp;
        //    auto frame = _source.alloc_frame(RS2_EXTENSION_MOTION_FRAME, data_size, additional_data, true);
        //    if (!frame)
        //    {
        //        LOG_INFO("Dropped frame. alloc_frame(...) returned nullptr");
        //        return;
        //    }
        //    frame->set_stream(request);

        //    std::vector<byte*> dest{const_cast<byte*>(frame->get_frame_data())};
        //    mode.unpacker->unpack(dest.data(),(const byte*)sensor_data.fo.pixels, mode.profile.width, mode.profile.height);

        //    if (_on_before_frame_callback)
        //    {
        //        auto callback = _source.begin_callback();
        //        auto stream_type = frame->get_stream()->get_stream_type();
        //        _on_before_frame_callback(stream_type, frame, std::move(callback));
        //    }

        //    _source.invoke_callback(std::move(frame));
        //});

        //_is_streaming = true;
    }

    void hid_sensor::stop()
    {
        std::lock_guard<std::mutex> lock(_configure_lock);
        if (!_is_streaming)
            throw wrong_api_call_sequence_exception("stop_streaming() failed. Hid device is not streaming!");


        _hid_device->stop_capture();
        _is_streaming = false;
        _source.flush();
        _source.reset();
        _hid_iio_timestamp_reader->reset();
        _custom_hid_timestamp_reader->reset();
        raise_on_before_streaming_changes(false);
    }

    std::vector<uint8_t> hid_sensor::get_custom_report_data(const std::string& custom_sensor_name,
        const std::string& report_name, platform::custom_sensor_report_field report_field) const
    {
        return _hid_device->get_custom_report_data(custom_sensor_name, report_name, report_field);
    }

    stream_profiles hid_sensor::init_stream_profiles()
    {
        stream_profiles stream_requests;
        for (auto it = _hid_sensors.rbegin(); it != _hid_sensors.rend(); ++it)
        {
            auto profiles = get_sensor_profiles(it->name);
            stream_requests.insert(stream_requests.end(), profiles.begin() ,profiles.end());
        }

        return stream_requests;
    }

    rs2_format sensor_base::fourcc_to_rs2_format(uint32_t fourcc_format) const
    {
        rs2_format f = RS2_FORMAT_ANY;
        try {
            f = _fourcc_to_rs2_format.at(fourcc_format);
        }
        catch (std::out_of_range)
        {
            //throw invalid_value_exception(to_string() << "rs2_format of fourcc " << fourcc_format << " not found!");
        }
        return f;
        //for (auto f_entry : _rs2_format_to_fourcc)
        //{
        //    auto rs2_fmt = f_entry.first;
        //    auto fourcc_fmt = f_entry.second;
        //    if (fourcc_format == fourcc_fmt)
        //        return rs2_fmt;
        //}

        //return RS2_FORMAT_ANY;
    }

    rs2_stream sensor_base::fourcc_to_rs2_stream(uint32_t fourcc_format) const
    {
        rs2_stream s = RS2_STREAM_ANY;
        try {
            s = _fourcc_to_rs2_stream.at(fourcc_format);
        }
        catch (std::out_of_range)
        {
            //throw invalid_value_exception(to_string() << "rs2_stream of fourcc " << fourcc_format << " not found!");
        }

        return s;
    }

    //rs2_format sensor_base::advanced_to_backend_format(rs2_format format) const
    //{
    //    rs2_format f = RS2_FORMAT_ANY;
    //    try {
    //        f = _advanced_to_backend_format.at(format);
    //    }
    //    catch (std::out_of_range)
    //    {
    //        //throw invalid_value_exception(to_string() << "rs2_stream of fourcc " << fourcc_format << " not found!");
    //    }

    //    return f;
    //}

    const std::string& hid_sensor::rs2_stream_to_sensor_name(rs2_stream stream) const
    {
        for (auto& elem : _sensor_name_and_hid_profiles)
        {
            if (stream == elem.second.stream)
                return elem.first;
        }
        throw invalid_value_exception("rs2_stream not found!");
    }

    uint32_t hid_sensor::stream_to_fourcc(rs2_stream stream) const
    {
        uint32_t fourcc;
        try{
            fourcc = stream_and_fourcc.at(stream);
        }
        catch(std::out_of_range)
        {
            throw invalid_value_exception(to_string() << "fourcc of stream " << rs2_stream_to_string(stream) << " not found!");
        }

        return fourcc;
    }

    uint32_t hid_sensor::fps_to_sampling_frequency(rs2_stream stream, uint32_t fps) const
    {
        // TODO: Add log prints
        auto it = _fps_and_sampling_frequency_per_rs2_stream.find(stream);
        if (it == _fps_and_sampling_frequency_per_rs2_stream.end())
            return fps;

        auto fps_mapping = it->second.find(fps);
        if (fps_mapping != it->second.end())
            return fps_mapping->second;
        else
            return fps;
    }

    uvc_sensor::uvc_sensor(std::string name, 
        std::shared_ptr<platform::uvc_device> uvc_device,
        std::unique_ptr<frame_timestamp_reader> timestamp_reader,
        device* dev)
       :   sensor_base(name, dev, (recommended_proccesing_blocks_interface*)this),
          _device(move(uvc_device)),
          //_sensor_owner(this->shared_from_this()),
          _user_count(0),
          _timestamp_reader(std::move(timestamp_reader))
    {
        register_metadata(RS2_FRAME_METADATA_BACKEND_TIMESTAMP,     make_additional_data_parser(&frame_additional_data::backend_timestamp));
    }

    iio_hid_timestamp_reader::iio_hid_timestamp_reader()
    {
        counter.resize(sensors);
        reset();
    }

    void iio_hid_timestamp_reader::reset()
    {
        std::lock_guard<std::recursive_mutex> lock(_mtx);
        started = false;
        for (auto i = 0; i < sensors; ++i)
        {
            counter[i] = 0;
        }
    }

    rs2_time_t iio_hid_timestamp_reader::get_frame_timestamp(std::shared_ptr<frame_interface> frame)
    {
        std::lock_guard<std::recursive_mutex> lock(_mtx);

        auto f = std::dynamic_pointer_cast<librealsense::frame>(frame);
        if (has_metadata(frame))
        {
            //  The timestamps conversions path comprise of:
            // FW TS (32bit) ->    USB Phy Layer (no changes)  -> Host Driver TS (Extend to 64bit) ->  LRS read as 64 bit
            // The flow introduces discrepancy with UVC stream which timestamps aer not extended to 64 bit by host driver both for Win and v4l backends.
            // In order to allow for hw timestamp-based synchronization of Depth and IMU streams the latter will be trimmed to 32 bit.
            // To revert to the extended 64 bit TS uncomment the next line instead
            //auto timestamp = *((uint64_t*)((const uint8_t*)fo.metadata));
            auto timestamp = (f->additional_data.metadata_size >= platform::hid_header_size) ?
                static_cast<uint32_t>(((platform::hid_header*)(f->additional_data.metadata_blob.data()))->timestamp) :
                    *((uint32_t*)((const uint8_t*)f->additional_data.metadata_blob.data()));

            // HID timestamps are aligned to FW Default - usec units
            return static_cast<rs2_time_t>(timestamp * TIMESTAMP_USEC_TO_MSEC);
        }

        if (!started)
        {
            LOG_WARNING("HID timestamp not found, switching to Host timestamps.");
            started = true;
        }

        return std::chrono::duration<rs2_time_t, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
    }

    bool iio_hid_timestamp_reader::has_metadata(std::shared_ptr<frame_interface> frame) const
    {
        auto f = std::dynamic_pointer_cast<librealsense::frame>(frame);

        if (f->additional_data.metadata_size > 0)
        {
            return true;
        }
        return false;
    }

    unsigned long long iio_hid_timestamp_reader::get_frame_counter(std::shared_ptr<frame_interface> frame) const
    {
        std::lock_guard<std::recursive_mutex> lock(_mtx);

        int index = 0;
        if (frame->get_stream()->get_stream_type() == RS2_STREAM_GYRO)
            index = 1;

        return ++counter[index];
    }

    rs2_timestamp_domain iio_hid_timestamp_reader::get_frame_timestamp_domain(std::shared_ptr<frame_interface> frame) const
    {
        if (has_metadata(frame))
        {
            return RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK;
        }
        return RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME;
    }


    //// TODO - Ariel - move to another file?
    //request::request(stream_profile sp)
    //{
    //    _resolved_request = resolve(sp);
    //}

    //stream_profile_base request::resolve(std::shared_ptr<stream_profile_interface> sp)
    //{
    //    // translate requests formats into fourcc formats
    //    stream_profile_base* resolved_req_ptr = dynamic_cast<stream_profile_base*>(sp.get());
    //    stream_profile_base resolved_req = resolved_req_ptr ? *resolved_req_ptr :
    //        throw invalid_value_exception("Trying to resolve an empty request.");

    //    auto&& const profiles = get_stream_profiles();

    //        for (auto&& const entry : _fourcc_to_rs2_format)
    //        {
    //            auto fourcc_format = entry.first;
    //            auto rs2_format = entry.second;
    //            if (rs2_format == req->get_format())
    //            {
    //                // TODO - think to change to O(n^2) instead of O(n^3)
    //                auto it = std::find_if(profiles.begin(), profiles.end(), [&fourcc_format](std::shared_ptr<stream_profile_interface> sp)
    //                {
    //                    auto backend_profile = dynamic_cast<backend_stream_profile*>(sp.get());
    //                    return fourcc_format == backend_profile->get_backend_profile().format;
    //                });

    //                if (it == profiles.end())
    //                    throw invalid_value_exception("Request format is not supported by the device!");

    //                auto profile = dynamic_cast<backend_stream_profile*>((*it).get())->get_backend_profile();
    //                resolved_req.push_back(profile);
    //                // TODO - handle request errors 
    //                //requests.erase(req);
    //            }
    //        }

    //    return resolved_req;


    //}

    synthetic_sensor::synthetic_sensor(std::string name, std::shared_ptr<sensor_base> sensor,
        device* device) : sensor_base(name, device, (recommended_proccesing_blocks_interface*)this), _raw_sensor(std::move(sensor))
    {}

    synthetic_sensor::~synthetic_sensor()
    {}

    option & synthetic_sensor::get_option(rs2_option id) const
    {
        return _raw_sensor->get_option(id);
    }

    void synthetic_sensor::register_option(rs2_option id, std::shared_ptr<option> option)
    {
        sensor_base::register_option(id, option); // TODO - Ariel - choose which sensor needs the options
        if (_raw_sensor)
            _raw_sensor->register_option(id, option);
    }

    void synthetic_sensor::unregister_option(rs2_option id)
    {
        sensor_base::unregister_option(id);
        if (_raw_sensor)
            _raw_sensor->unregister_option(id);
    }

    void synthetic_sensor::sort_profiles_by_resolution(stream_profiles* profiles)
    {
        // by height
        std::sort(profiles->begin(), profiles->end(), [](std::shared_ptr<stream_profile_interface> spia, std::shared_ptr<stream_profile_interface> spib)
        {
            auto spa = std::dynamic_pointer_cast<video_stream_profile>(spia);
            auto spb = std::dynamic_pointer_cast<video_stream_profile>(spib);
            return spa->get_height() < spb->get_height();
        });

        // by width
        std::sort(profiles->begin(), profiles->end(), [](std::shared_ptr<stream_profile_interface> spia, std::shared_ptr<stream_profile_interface> spib)
        {
            auto spa = std::dynamic_pointer_cast<video_stream_profile>(spia);
            auto spb = std::dynamic_pointer_cast<video_stream_profile>(spib);
            return spa->get_width() < spb->get_width();
        });

        // by format
        std::sort(profiles->begin(), profiles->end(), [](std::shared_ptr<stream_profile_interface> spia, std::shared_ptr<stream_profile_interface> spib)
        {
            return spia->get_format() < spib->get_format();
        });
    }

    stream_profiles synthetic_sensor::init_stream_profiles()
    {
        stream_profiles result_profiles;
        //profiles = raw.get_profiles();
        auto profiles = _raw_sensor->get_stream_profiles();

        // cache each profile to a single processing block factory for an easy one to one mapping.
        //std::unordered_map<std::shared_ptr<stream_profile_interface>, processing_block_factory> profile_to_pb_factory;

        //foreach pbf:
        for (auto&& pbf : _pb_factories)
        {
            //    in = pbf.input
            auto sources = pbf.get_source_formats();
            auto targets = pbf.get_target_formats();

            for (auto&& source_fmt : sources)
            {
                // add profiles that are supported by the device
                for (auto&& profile : profiles)
                {
                    auto vsp = std::dynamic_pointer_cast<video_stream_profile>(profile);
                    if (profile->get_format() == source_fmt)
                    {
                        for (auto&& target : targets)
                        {
                            target.fps = profile->get_framerate();
                            target.height = vsp->get_height();
                            target.width = vsp->get_width();

                            auto cloned_profile = std::make_shared<video_stream_profile>(vsp->get_backend_profile());
                            //cloned_profile->set_unique_id(profile->get_unique_id());
                            cloned_profile->set_dims(target.width, target.height);
                            cloned_profile->set_format(target.format);
                            cloned_profile->set_stream_index(target.index);
                            cloned_profile->set_stream_type(target.stream);
                            cloned_profile->set_framerate(target.fps);

                            // cache the source to target mapping
                            _source_to_target_profiles_map[profile].push_back(cloned_profile);
                            _target_to_source_profiles_map[target].push_back(profile);

                            //auto profile_to_pb_factory_match = std::find_if(profile_to_pb_factory.begin(), profile_to_pb_factory.end(), [&profile](auto entry) {
                            //    return profile == entry.first;
                            //});
                            //if (profile_to_pb_factory_match != profile_to_pb_factory.end())
                            //{
                            //    // profile found in cache
                            //    // cache target to source profile only if it is the same processing block which defined it first
                            //    if (pbf == profile_to_pb_factory_match->second)
                            //        _target_to_source_profiles_map[cloned_profile].push_back(profile);
                            //}
                            //else
                            //{
                            //    // profile not found in cache, add it
                            //    profile_to_pb_factory[profile] = pbf;
                            //    _target_to_source_profiles_map[cloned_profile].push_back(profile);
                            //}

                            // disregard duplicated from profiles list
                            if (std::find_if(result_profiles.begin(), result_profiles.end(), [&cloned_profile](std::shared_ptr<stream_profile_interface> spi)
                            {
                                auto sp = std::dynamic_pointer_cast<video_stream_profile>(spi);
                                return (sp->get_format() == cloned_profile->get_format() &&
                                    //sp->get_unique_id() == p->get_unique_id() &&
                                    sp->get_stream_index() == cloned_profile->get_stream_index() &&
                                    sp->get_stream_type() == cloned_profile->get_stream_type() &&
                                    sp->get_height() == cloned_profile->get_height() &&
                                    sp->get_framerate() == cloned_profile->get_framerate() &&
                                    sp->get_width() == cloned_profile->get_width());
                            }) != result_profiles.end())
                                continue;

                            result_profiles.push_back(cloned_profile);

                            //_source_to_target_profiles_map[profile][target.format] = cloned_profile;
                            //_target_to_source_profiles_map[cloned_profile] = profile;
                        }
                    }
                }
            }
            
            //    out = pbf.outputs
            //
            //    foreach in:
            //        foreach p in profiles:
            //            if p matches in:
            //                foreach out:
            //                    O = clone out
            //                    foreach feild in p
            //                        if not placeholder
            //                            O.feild = p.feild
            //                    add O
        }
        _owner->tag_profiles(result_profiles);
        sort_profiles_by_resolution(&result_profiles);
        return result_profiles;
    }

    std::pair<processing_block_factory, stream_profiles> synthetic_sensor::find_requests_best_match(stream_profiles requests)
    {
        // best fitting processing block is defined as the processing block which its sources
        // covers the maximum amount of requests given.

        stream_profiles best_match_requests;
        stream_profiles current_best_match_requests;

        processing_block_factory best_match_processing_block_factory;

        for (auto&& pbf : _pb_factories)
        {
            current_best_match_requests = pbf.find_satisfied_requests(requests);
            if (current_best_match_requests.size() > best_match_requests.size())
            {
                best_match_requests = current_best_match_requests;
                best_match_processing_block_factory = pbf;
            }
        }

        return {best_match_processing_block_factory, best_match_requests};
    }

    std::unordered_set<std::shared_ptr<stream_profile_interface>> synthetic_sensor::map_requests_to_source_profiles(stream_profiles requests)
    {
        std::unordered_set<std::shared_ptr<stream_profile_interface>> mapped_source_profiles;
        for (auto req : requests)
        {
            stream_output_info output_info(req);
            auto mapped_profiles = _target_to_source_profiles_map[output_info];
            mapped_source_profiles.insert(begin(mapped_profiles), end(mapped_profiles));
        }

        return mapped_source_profiles;
    }

    stream_profiles synthetic_sensor::resolve_requests(const stream_profiles& requests)
    {
        std::unordered_set<std::shared_ptr<stream_profile_interface>> resolved_req_set;
        stream_profiles resolved_req;
        stream_profiles unhandled_reqs(requests);
        
        /////////////////////////////////////////////
        
        // cache the requests
        for (auto req : requests)
        {
            cached_requests[req->get_format()].push_back(req);
        }
        
        // while not finished handling all of the requests do
        while (!unhandled_reqs.empty())
        {
            // union the mapped request to source profiles
            auto mapped_source_profiles = map_requests_to_source_profiles(unhandled_reqs);

            // find the best fitting processing block - the one which resolves the most requests.
            auto best_match_mapped = find_requests_best_match({ mapped_source_profiles.begin(), mapped_source_profiles.end() });
            auto best_pb = best_match_mapped.first;
            auto best_mapped_reqs = best_match_mapped.second;
            auto best_pb_sources = best_pb.get_source_formats();

            // generate this processing block and append it to the cached processing blocks.
            _formats_to_processing_block[best_pb_sources] = best_pb.generate_processing_block();
            
            // mark as handled resolved requests
            resolved_req_set.insert(begin(best_mapped_reqs), end(best_mapped_reqs));
            for (auto mapped_req : best_mapped_reqs)
            {
                auto reqs = _source_to_target_profiles_map[mapped_req];
                for (auto req : reqs)
                {
                    auto unhandled_req = std::find_if(unhandled_reqs.begin(), unhandled_reqs.end(), [&req](auto sp) {
                        return req == sp;
                    });
                    if (unhandled_req != end(unhandled_reqs))
                        unhandled_reqs.erase(unhandled_req);
                }
            }
        }
        
        resolved_req = { resolved_req_set.begin(), resolved_req_set.end() };
        return resolved_req;
    }

    void synthetic_sensor::open(const stream_profiles& requests)
    {
        std::lock_guard<std::mutex> lock(_configure_lock);
        auto resolved_req = resolve_requests(requests);

        auto raw_sensor = std::dynamic_pointer_cast<uvc_sensor>(_raw_sensor);
        raw_sensor->set_owner_sensor(this->shared_from_this()); // TODO - Ariel - maybe move set_owner_sensor method to sensor_base if it is common for hid and uvc

        _raw_sensor->open(resolved_req);
    }

    void synthetic_sensor::close()
    {
        std::lock_guard<std::mutex> lock(_configure_lock);
        _raw_sensor->close();
        // TODO - Ariel - reset stream to processing block map
    }

    template<class T>
    frame_callback_ptr make_callback(T callback)
    {
        return {
            new internal_frame_callback<T>(callback),
            [](rs2_frame_callback* p) { /*p->release(); */}
        };
    }

    std::shared_ptr<stream_profile_interface> synthetic_sensor::filter_frame_by_requests(frame_holder& f)
    {
        std::shared_ptr<stream_profile_interface> cached_profile;
        auto cached_req = cached_requests.find(f->get_stream()->get_format());
        if (cached_req == cached_requests.end())
            return cached_profile;

        // find a match between the request and the processed frame
        for (auto req : cached_req->second)
        {
            if (req->get_stream_index() == f->get_stream()->get_stream_index())
            {
                cached_profile = req;
                break;
            }
        }

        return cached_profile;
    }

    void synthetic_sensor::start(frame_callback_ptr callback)
    {
        std::lock_guard<std::mutex> lock(_configure_lock);

        //map<format, sp> cached;

        auto output_cb = make_callback([&, callback](frame_holder f) {
            // Filter
            //if not found cached[f.format] return;
            auto cached_profile = filter_frame_by_requests(f);

            if (cached_profile)
            {
                f.frame->acquire();
                f->set_stream(cached_profile);
            }
            else
                return;

            callback->on_frame((rs2_frame*)f.frame);
        });

        for (auto&& pb_entry : _formats_to_processing_block)
        {
            auto&& pb = pb_entry.second;
            if (pb)
            {
                pb_entry.second->set_output_callback(output_cb);
            }
        }

        auto process_cb = make_callback([&, callback, this](frame_holder f) {

            //std::lock_guard<std::mutex> lock(_configure_lock); // TODO - Ariel - maybe can cause deadlock?

            // cache profile data for post-processing re-definition
            //cached_profile = f.frame->get_stream()->clone();

            for (auto&& pb_entry : _formats_to_processing_block)
            {
                if (!f)
                    return;
                // TODO - Ariel - handle multiple requests
                auto&& pb = pb_entry.second;
                auto&& requested_fmt = pb_entry.first;

                //cached = all_cached[f.frame->get_stream()];
                //cached_profile = _source_to_target_profiles_map[f->get_stream()][requested_fmt];

                // process only if the frame format matches the processing block source format
                if (std::find_if(requested_fmt.begin(), requested_fmt.end(), [&f](auto fmt) 
                {
                    return f->get_stream()->get_format() == fmt;
                }) == requested_fmt.end())
                    continue;
                //if (f->get_stream()->get_format() != requested_fmt)
                //    continue;

                //if (pb)
                //{
                pb->invoke(std::move(f));
                //}
                //else
                //{
                //    f.frame->acquire();
                //    //f->set_stream(cached_profile);
                //    callback->on_frame((rs2_frame*)f.frame);
                //}

                // completed handling frame
                return;
            }
        });

        // call the processing block on the frame
        _raw_sensor->start(process_cb);

        //_raw_sensor->start(callback);
    }

    void synthetic_sensor::stop()
    {
        std::lock_guard<std::mutex> lock(_configure_lock);
        _raw_sensor->stop();
        cached_requests.erase(cached_requests.begin(), cached_requests.end());
    }

    void synthetic_sensor::register_processing_block(std::vector<rs2_format> from, std::vector<stream_output_info> to, std::function<std::shared_ptr<processing_block>(void)> generate_func)
    {
        processing_block_factory pbf(from, to, generate_func);
        _pb_factories.push_back(pbf);
    }

    //template <rs2_extension E, typename P>
    //bool synthetic_sensor::extend_to_aux(P* p, void** ext)
    //{
    //    using EXT_TYPE = typename ExtensionToType<E>::type;
    //    auto ptr = As<EXT_TYPE>(p);
    //    if (!ptr)
    //        return false;

    //    *ext = ptr;
    //    return true;
    //}

    //// TODO - Ariel - fix the extend_to func
    //bool synthetic_sensor::extend_to(rs2_extension extension_type, void** ext)
    //{
    //    switch (extension_type)
    //    {
    //    case RS2_EXTENSION_OPTIONS: // [[fallthrough]]
    //    case RS2_EXTENSION_INFO:    // [[fallthrough]]
    //        *ext = this;
    //        return true;
    //    case RS2_EXTENSION_DEPTH_SENSOR: return extend_to_aux<RS2_EXTENSION_DEPTH_SENSOR>(_raw_sensor.get(), ext);
    //    case RS2_EXTENSION_DEPTH_STEREO_SENSOR: return extend_to_aux<RS2_EXTENSION_DEPTH_STEREO_SENSOR>(_raw_sensor.get(), ext);
    //    case RS2_EXTENSION_POSE_SENSOR: return extend_to_aux<RS2_EXTENSION_POSE_SENSOR>(_raw_sensor.get(), ext);
    //        //Other extensions are not expected to be extensions of a sensor
    //    default:
    //        LOG_WARNING("Extensions type is unhandled: " << get_string(extension_type));
    //        return false;
    //    }
    //}

    processing_block_factory::processing_block_factory(std::vector<rs2_format> from, std::vector<stream_output_info> to, std::function<std::shared_ptr<processing_block>(void)> generate_func) :
        _source_formats(from), _target_formats(to), generate_processing_block(generate_func)
    {
    }

    processing_block_factory::processing_block_factory(const processing_block_factory & rhs)
    {
        copy_processing_block_factory(rhs);
    }

    processing_block_factory& processing_block_factory::operator=(const processing_block_factory & rhs)
    {
        if (&rhs == this)
            return *this;

        copy_processing_block_factory(rhs);

        return *this;
    }

    bool processing_block_factory::operator==(const processing_block_factory & rhs)
    {

        auto&& rhs_src_fmts = rhs.get_source_formats();
        for (auto src : _source_formats)
        {
            if (std::find_if(rhs_src_fmts.begin(), rhs_src_fmts.end(), [&src](auto fmt) {
                return src == fmt;
            }) == rhs_src_fmts.end())
                return false;
        }

        auto&& rhs_tgt_fmts = rhs.get_target_formats();
        for (auto tgt : _target_formats)
        {
            if (std::find_if(rhs_tgt_fmts.begin(), rhs_tgt_fmts.end(), [&tgt](auto rhs_tgt) {
                return tgt == rhs_tgt;
            }) == rhs_tgt_fmts.end())
                return false;
        }

        return true;
    }

    stream_profiles processing_block_factory::find_satisfied_requests(stream_profiles requests)
    {
        // Return all requests which satisfies the processing block.
        // a processing block is satisfied, if ALL of its sources found a match with a request.

        if (requests.size() < _source_formats.size())
            return stream_profiles();

        stream_profiles satisfied_req;
        auto srcs = _source_formats;
        for (auto&& req : requests)
        {
            if (std::find_if(begin(srcs), end(srcs), [&req](auto src) {
                return req->get_format() == src;
            }) != end(srcs))
                satisfied_req.push_back(req);
        }
        return satisfied_req;
    }

    void processing_block_factory::copy_processing_block_factory(const processing_block_factory & rhs)
    {
        _source_formats = rhs.get_source_formats();
        _target_formats = rhs.get_target_formats();
        generate_processing_block = rhs.generate_processing_block;
    }

    stream_output_info::stream_output_info(std::shared_ptr<stream_profile_interface> sp)
    {
        width = 0;
        height = 0;
        auto vsp = std::dynamic_pointer_cast<video_stream_profile>(sp);
        if (vsp)
        {
            height = vsp->get_height();
            width = vsp->get_width();
        }
        fps = sp->get_framerate();
        format = sp->get_format();
        stream = sp->get_stream_type();
        index = sp->get_stream_index();
    }

    stream_output_info::stream_output_info(const stream_output_info & other)
    {
        copy(other);
    }

    bool stream_output_info::operator==(const stream_output_info & rhs)
    {
        return rhs.format == format &&
            rhs.stream == stream &&
            rhs.width == width &&
            rhs.height == height &&
            rhs.fps == fps &&
            rhs.index == index;
    }

    void stream_output_info::copy(const stream_output_info & other)
    {
        format = other.format;
        stream = other.stream;
        width = other.width;
        height = other.height;
        index = other.index;
        fps = other.fps;
    }

    stream_output_info & stream_output_info::operator=(const stream_output_info & rhs)
    {
        if (this == &rhs)
            return *this;

        copy(rhs);
        return *this;
    }

    bool operator<(const stream_output_info & lhs, const stream_output_info & rhs)
    {
        return lhs.format < rhs.format ||
            lhs.index < rhs.index ||
            lhs.width < rhs.width ||
            lhs.height < rhs.height ||
            lhs.fps < rhs.fps ||
            lhs.stream < rhs.stream;
    }
    bool operator==(const stream_output_info & lhs, const stream_output_info & rhs)
    {
        return rhs.format == lhs.format &&
            rhs.stream == lhs.stream &&
            rhs.width == lhs.width &&
            rhs.height == lhs.height &&
            rhs.fps == lhs.fps &&
            rhs.index == lhs.index;
    }
}
