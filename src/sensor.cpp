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

    //stream_profiles sensor_base::resolve_requests(stream_profiles requests)
    //{
    //    //// translate requests formats into fourcc formats
    //    //stream_profiles resolved_requests;
    //    //auto&& const profiles = get_stream_profiles();

    //    //for (auto req : requests)
    //    //{
    //    //    for (auto&& const entry : _fourcc_to_rs2_format)
    //    //    {
    //    //        auto fourcc_format = entry.first;
    //    //        auto rs2_format = entry.second;
    //    //        if (rs2_format == req->get_format())
    //    //        {
    //    //            //// TODO - think to change to O(n^2) instead of O(n^3)
    //    //            //auto profile_it = std::find_if(profiles.begin(), profiles.end(), [&fourcc_format, &resolved_req](std::shared_ptr<stream_profile_interface> sp)
    //    //            //{
    //    //            //    auto profile = std::dynamic_pointer_cast<stream_profile_base>(sp);
    //    //            //    if (fourcc_format == profile->get_backend_profile().format)
    //    //            //    {
    //    //            //        resolved_req.push_back(profile);
    //    //            //        return true;
    //    //            //    }
    //    //            //    return false;
    //    //            //});

    //    //            //if (profile_it == profiles.end())
    //    //            //    throw invalid_value_exception("Request format is not supported by the device!");

    //    //            //// TODO - Ariel - maybe convert given requests into resolved ones
    //    //            //auto base_req = std::dynamic_pointer_cast<video_stream_profile>(req);
    //    //            //auto resolved_req = std::make_shared<video_stream_profile>(base_req->get_backend_profile());
    //    //            //resolved_req->set_format(advanced_to_backend_format(rs2_format));
    //    //            //resolved_req->set_stream_type(base_req->get_stream_type());
    //    //            //resolved_requests.push_back(resolved_req);
    //    //            //req->set_format(advanced_to_backend_format(rs2_format));
    //    //            //resolved_requests.push_back(req);
    //    //            resolved_requests.push_back(req);

    //    //            //auto profile = dynamic_cast<backend_stream_profile*>((*it).get())->get_backend_profile();
    //    //            //resolved_req.push_back(*profile_it);
    //    //            // TODO - handle request errors 
    //    //            //requests.erase(req);
    //    //        }
    //    //    }
    //    //}

    //    //return resolved_requests;
    //}

    //std::vector<request_mapping> sensor_base::resolve_requestss(stream_profiles requests)
    //{
    //    // per requested profile, find all 4ccs that support that request.
    //    std::map<int, std::set<uint32_t>> legal_fourccs;
    //    auto profiles = get_stream_profiles();
    //    for (auto&& r : requests)
    //    {
    //        auto sp = to_profile(r.get());
    //        for (auto&& mode : profiles)
    //        {
    //            if (auto backend_profile = dynamic_cast<backend_stream_profile*>(mode.get()))
    //            {
    //                auto m = to_profile(mode.get());

    //                if (m.fps == sp.fps && m.height == sp.height && m.width == sp.width)
    //                    legal_fourccs[sp.index].insert(backend_profile->get_backend_profile().format); // TODO: Stread ID???
    //            }
    //        }
    //    }

    //    //if you want more efficient data structure use std::unordered_set
    //    //with well-defined hash function
    //    std::set<request_mapping> results;
    //    while (!requests.empty() && !_pixel_formats.empty())
    //    {
    //        auto max = 0;
    //        size_t best_size = 0;
    //        auto best_pf = &_pixel_formats.front();
    //        auto best_unpacker = &_pixel_formats.front().unpackers.front();
    //        platform::stream_profile uvc_profile{};
    //        for (auto&& pf : _pixel_formats)
    //        {
    //            // Speeds up algorithm by skipping obviously useless 4ccs
    //            // if (std::none_of(begin(legal_fourccs), end(legal_fourccs), [&](const uint32_t fourcc) {return fourcc == pf.fourcc; })) continue;

    //            for (auto&& unpacker : pf.unpackers)
    //            {
    //                auto count = static_cast<int>(std::count_if(begin(requests), end(requests),
    //                    [&pf, &legal_fourccs, &unpacker, this](const std::shared_ptr<stream_profile_interface>& r)
    //                {
    //                    // only count if the 4cc can be unpacked into the relevant stream/format
    //                    // and also, the pixel format can be streamed in the requested dimensions/fps.
    //                    return unpacker.satisfies(to_profile(r.get()), pf.fourcc, _uvc_profiles) && legal_fourccs[r->get_stream_index()].count(pf.fourcc);
    //                }));

    //                // Here we check if the current pixel format / unpacker combination is better than the current best.
    //                // We judge on two criteria. A: how many of the requested streams can we supply? B: how many total streams do we open?
    //                // Optimally, we want to find a combination that supplies all the requested streams, and no additional streams.
    //                if (
    //                    count > max                                 // If the current combination supplies more streams, it is better.
    //                    || (count == max                            // Alternatively, if it supplies the same number of streams,
    //                        && unpacker.outputs.size() < best_size) // but this combination opens fewer total streams, it is also better
    //                    )
    //                {
    //                    max = count;
    //                    best_size = unpacker.outputs.size();
    //                    best_pf = &pf;
    //                    best_unpacker = &unpacker;
    //                }
    //            }
    //        }

    //        if (max == 0) break;

    //        requests.erase(std::remove_if(begin(requests), end(requests),
    //            [best_unpacker, best_pf, &results, &legal_fourccs, this](const std::shared_ptr<stream_profile_interface>& r)
    //        {
    //            if (best_unpacker->satisfies(to_profile(r.get()), best_pf->fourcc, _uvc_profiles) && legal_fourccs[r->get_stream_index()].count(best_pf->fourcc))
    //            {
    //                auto request = dynamic_cast<const video_stream_profile*>(r.get());

    //                request_mapping mapping;
    //                mapping.unpacker = best_unpacker;
    //                mapping.pf = best_pf;
    //                auto uvc_profile = best_unpacker->get_uvc_profile(to_profile(r.get()), best_pf->fourcc, _uvc_profiles);
    //                if (!request) {
    //                    mapping.profile = { 0, 0, r->get_framerate(), best_pf->fourcc };
    //                }
    //                else
    //                {
    //                    mapping.profile = { uvc_profile.width, uvc_profile.height, uvc_profile.fps, best_pf->fourcc };
    //                }

    //                results.insert(mapping);

    //                auto it = results.find(mapping);
    //                if (it != results.end())
    //                {
    //                    it->original_requests.push_back(map_requests(r));
    //                }

    //                return true;
    //            }
    //            return false;
    //        }), end(requests));
    //    }

    //    if (requests.empty()) return{ begin(results), end(results) };

    //    throw invalid_value_exception("Subdevice unable to satisfy stream requests!");
    //}

    //std::shared_ptr<stream_profile_interface> sensor_base::map_requests(std::shared_ptr<stream_profile_interface> request)
    //{
    //    stream_profiles results;
    //    auto profiles = get_stream_profiles();

    //    auto it = std::find_if(profiles.begin(), profiles.end(), [&](std::shared_ptr<stream_profile_interface> p) {
    //        return to_profile(p.get()) == to_profile(request.get());
    //    });

    //    if (it == profiles.end())
    //        throw invalid_value_exception("Subdevice could not map requests!");

    //    return *it;
    //}

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
            //profile->set_intrinsics([]()
            //{
            //    return rs2_intrinsics{};
            //});
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

    std::shared_ptr<frame> sensor_base::generate_frame_from_data(const platform::frame_object& fo,
        frame_timestamp_reader* timestamp_reader,
        const rs2_time_t& last_timestamp,
        const unsigned long long& last_frame_number,
        std::shared_ptr<stream_profile_interface> profile)
    {
        auto system_time = environment::get_instance().get_time_service()->get_time();
        auto fr = std::make_shared<frame>();
        byte* pix = (byte*)fo.pixels;
        std::vector<byte> pixels(pix, pix + fo.frame_size);
        fr->data = pixels;
        fr->set_stream(profile);

        // generate additional data
        frame_additional_data additional_data(0,
            0,
            system_time,
            static_cast<uint8_t>(fo.metadata_size),
            (const uint8_t*)fo.metadata,
            fo.backend_time,
            last_timestamp,
            last_frame_number,
            false);
        fr->additional_data = additional_data;

        // update additional data
        additional_data.timestamp = timestamp_reader->get_frame_timestamp(fr);
        additional_data.last_frame_number = last_frame_number;
        additional_data.frame_number = timestamp_reader->get_frame_counter(fr);
        fr->additional_data = additional_data;

        return fr;
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
        _source.set_sensor(_sensor_owner);

        std::vector<platform::stream_profile> commited;

        for (auto&& req_profile : requests)
        {
            auto req_profile_base = std::dynamic_pointer_cast<stream_profile_base>(req_profile);
            try
            {
                unsigned long long last_frame_number = 0;
                rs2_time_t last_timestamp = 0;
                _device->probe_and_commit(req_profile_base->get_backend_profile(),
                    [this, req_profile_base, req_profile, last_frame_number, last_timestamp](platform::stream_profile p, platform::frame_object f, std::function<void()> continuation) mutable
                {
                    auto system_time = environment::get_instance().get_time_service()->get_time();
                    auto fr = generate_frame_from_data(f, _timestamp_reader.get(), last_timestamp, last_frame_number, req_profile_base);
                    auto requires_processing = true; // TODO - Ariel add option
                    auto frame_counter = fr->additional_data.frame_number;
                    auto timestamp_domain = _timestamp_reader->get_frame_timestamp_domain(fr);
                    auto timestamp = fr->additional_data.timestamp;
                    auto bpp = get_image_bpp(req_profile_base->get_format());

                    if (!this->is_streaming())
                    {
                        LOG_WARNING("Frame received with streaming inactive,"
                            << librealsense::get_string(req_profile_base->get_stream_type())
                            << req_profile_base->get_stream_index()
                            << ", Arrived," << std::fixed << f.backend_time << " " << system_time);
                        return;
                    }

                    frame_continuation release_and_enqueue(continuation, f.pixels);
                    
                    LOG_DEBUG("FrameAccepted," << librealsense::get_string(req_profile_base->get_stream_type())
                        << ",Counter," << std::dec << fr->additional_data.frame_number
                            << ",Index," << req_profile_base->get_stream_index()
                            << ",BackEndTS," << std::fixed << f.backend_time
                            << ",SystemTime," << std::fixed << system_time
                            << " ,diff_ts[Sys-BE]," << system_time - f.backend_time
                            << ",TS," << std::fixed << timestamp << ",TS_Domain," << rs2_timestamp_domain_to_string(timestamp_domain)
                            << ",last_frame_number," << last_frame_number << ",last_timestamp," << last_timestamp);

                    last_frame_number = frame_counter;
                    last_timestamp = timestamp;

                    auto vsp = As<video_stream_profile, stream_profile_interface>(req_profile);
                    int width = vsp ? vsp->get_width() : 0;
                    int height = vsp ? vsp->get_height() : 0;

                    frame_holder fh = _source.alloc_frame(stream_to_frame_types(req_profile_base->get_stream_type()), width * height * bpp / 8, fr->additional_data, requires_processing);
                    if (fh.frame)
                    {
                        memcpy((void*)fh->get_frame_data(), fr->data.data(), sizeof(byte)*fr->data.size());
                        auto video = (video_frame*)fh.frame;
                        video->assign(width, height, width * bpp / 8, bpp);
                        video->set_timestamp_domain(timestamp_domain);
                        fh->set_stream(req_profile_base);
                    }
                    else
                    {
                        LOG_INFO("Dropped frame. alloc_frame(...) returned nullptr");
                        return;
                    }

                    if (!requires_processing)
                    {
                        fh->attach_continuation(std::move(release_and_enqueue));
                    }

                    if (fh->get_stream().get())
                    {
                        _source.invoke_callback(std::move(fh));
                    }
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

        _hid_device->register_profiles(profiles_vector);
        for (auto& elem : _hid_device->get_sensors())
            _hid_sensors.push_back(elem);
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

    /*void hid_sensor::opens(const stream_profiles& requests)
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
    }*/

    /*stream_profiles hid_sensor::resolve_requests(stream_profiles requests)
    {
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
                                       request->get_format() }));
                    _is_configured_stream[request->get_stream_type()] = true;
                    _hid_mapping.insert(std::make_pair(sensor_name, map));
                }
            }
        }
    }*/

    void hid_sensor::open(const stream_profiles& requests)
    {
        std::lock_guard<std::mutex> lock(_configure_lock);
        if (_is_streaming)
            throw wrong_api_call_sequence_exception("open(...) failed. Hid device is streaming!");
        else if (_is_opened)
            throw wrong_api_call_sequence_exception("Hid device is already opened!");

        std::vector<platform::hid_profile> configured_hid_profiles;
        for (auto& request : requests)
        {
            auto sensor_name = rs2_stream_to_sensor_name(request->get_stream_type());
            //stream_info info = { request->get_format(), request->get_stream_type(), 0, 0, 0,
            //    fps_to_sampling_frequency(request->get_stream_type(), request->get_framerate()) };
            _configured_profiles.insert(std::make_pair(sensor_name, request));
            _is_configured_stream[request->get_stream_type()] = true;
            configured_hid_profiles.push_back(platform::hid_profile{ sensor_name,
                fps_to_sampling_frequency(request->get_stream_type(), request->get_framerate()) });
        }
        //for (auto& elem : _configured_profiles)
        //{
        //    configured_hid_profiles.push_back(platform::hid_profile{ elem.first, elem.second.fps });
        //}
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
        std::lock_guard<std::mutex> lock(_configure_lock);
        if (_is_streaming)
            throw wrong_api_call_sequence_exception("start_streaming(...) failed. Hid device is already streaming!");
        else if(!_is_opened)
            throw wrong_api_call_sequence_exception("start_streaming(...) failed. Hid device was not opened!");

        _source.set_callback(callback);
        _source.init(_metadata_parsers);
        _source.set_sensor(_sensor_owner);

        unsigned long long last_frame_number = 0;
        rs2_time_t last_timestamp = 0;
        raise_on_before_streaming_changes(true); //Required to be just before actual start allow recording to work

        _hid_device->start_capture([this, last_frame_number, last_timestamp](const platform::sensor_data& sensor_data) mutable
        {
            auto system_time = environment::get_instance().get_time_service()->get_time();
            auto timestamp_reader = _hid_iio_timestamp_reader.get();
            static const std::string custom_sensor_name = "custom";
            auto sensor_name = sensor_data.sensor.name;
            auto request = _configured_profiles[sensor_name];
            bool is_custom_sensor = false;
            static const uint32_t custom_source_id_offset = 16;
            uint8_t custom_gpio = 0;
            auto custom_stream_type = RS2_STREAM_ANY;
            if (sensor_name == custom_sensor_name)
            {
                custom_gpio = *(reinterpret_cast<uint8_t*>((uint8_t*)(sensor_data.fo.pixels) + custom_source_id_offset));
                custom_stream_type = custom_gpio_to_stream_type(custom_gpio);

                if (!_is_configured_stream[custom_stream_type])
                {
                    LOG_DEBUG("Unrequested " << rs2_stream_to_string(custom_stream_type) << " frame was dropped.");
                    return;
                }

                is_custom_sensor = true;
                timestamp_reader = _custom_hid_timestamp_reader.get();
            }

            if (!this->is_streaming())
            {
                auto stream_type = request->get_stream_type();
                LOG_INFO("HID Frame received when Streaming is not active,"
                    << get_string(stream_type)
                    << ",Arrived," << std::fixed << system_time);
                return;
            }

            //////////////////////////////////
            auto fr = generate_frame_from_data(sensor_data.fo, timestamp_reader, last_timestamp, last_frame_number, request);
            auto frame_counter = fr->additional_data.frame_number;
            auto timestamp_domain = timestamp_reader->get_frame_timestamp_domain(fr);
            auto timestamp = fr->additional_data.timestamp;
            auto bpp = get_image_bpp(request->get_format());
            //////////////////////////////////

            auto data_size = sensor_data.fo.frame_size;
            //mode.profile.width = (uint32_t)data_size;
            //mode.profile.height = 1;

            // Determine the timestamp for this HID frame
            //auto timestamp = timestamp_reader->get_frame_timestamp(fr);
            //auto frame_counter = timestamp_reader->get_frame_counter(fr);
            //auto ts_domain = timestamp_reader->get_frame_timestamp_domain(fr);

            //frame_additional_data additional_data(timestamp,
            //    frame_counter,
            //    system_time,
            //    static_cast<uint8_t>(sensor_data.fo.metadata_size),
            //    (const uint8_t*)sensor_data.fo.metadata,
            //    sensor_data.fo.backend_time,
            //    last_timestamp,
            //    last_frame_number,
            //    false);

            //additional_data.timestamp_domain = ts_domain;
            //additional_data.backend_timestamp = sensor_data.fo.backend_time;



            LOG_DEBUG("FrameAccepted," << get_string(request->get_stream_type())
                << ",Counter," << std::dec << frame_counter << ",Index,0"
                << ",BackEndTS," << std::fixed << sensor_data.fo.backend_time
                << ",SystemTime," << std::fixed << system_time
                << " ,diff_ts[Sys-BE]," << system_time - sensor_data.fo.backend_time
                << ",TS," << std::fixed << timestamp << ",TS_Domain," << rs2_timestamp_domain_to_string(timestamp_domain)
                << ",last_frame_number," << last_frame_number << ",last_timestamp," << last_timestamp);

            last_frame_number = frame_counter;
            last_timestamp = timestamp;
            frame_holder frame = _source.alloc_frame(RS2_EXTENSION_MOTION_FRAME, data_size, fr->additional_data, true);
            memcpy((void*)frame->get_frame_data(), fr->data.data(), sizeof(byte)*fr->data.size());
            if (!frame)
            {
                LOG_INFO("Dropped frame. alloc_frame(...) returned nullptr");
                return;
            }
            frame->set_stream(request);

            //std::vector<byte*> dest{ const_cast<byte*>(frame->get_frame_data()) };
            //mode.unpacker->unpack(dest.data(), (const byte*)sensor_data.fo.pixels, mode.profile.width, mode.profile.height);

            //if (_on_before_frame_callback)
            //{
            //    auto callback = _source.begin_callback();
            //    auto stream_type = frame->get_stream()->get_stream_type();
            //    _on_before_frame_callback(stream_type, frame, std::move(callback));
            //}

            _source.invoke_callback(std::move(frame));
        });
        _is_streaming = true;
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

        _fourcc_to_rs2_format = {
            {rs_fourcc('Y','U','Y','2'), RS2_FORMAT_YUYV},
            {rs_fourcc('G','R','E','Y'), RS2_FORMAT_Y8},
            {rs_fourcc('Y','8','I',' '), RS2_FORMAT_Y8I},
            {rs_fourcc('Y','1','6',' '), RS2_FORMAT_Y16},
            {rs_fourcc('Z','1','6',' '), RS2_FORMAT_Z16},
            {rs_fourcc('C',' ',' ',' '), RS2_FORMAT_RAW8}
        };

        _fourcc_to_rs2_stream = {
            {rs_fourcc('Y','U','Y','2'), RS2_STREAM_COLOR},
            {rs_fourcc('G','R','E','Y'), RS2_STREAM_INFRARED},
            {rs_fourcc('Y','8','I',' '), RS2_STREAM_INFRARED},
            {rs_fourcc('Y','1','6',' '), RS2_STREAM_INFRARED},
            {rs_fourcc('Z','1','6',' '), RS2_STREAM_DEPTH},
            {rs_fourcc('C',' ',' ',' '), RS2_STREAM_CONFIDENCE},
        };
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

    void synthetic_sensor::sort_profiles(stream_profiles* profiles)
    {
        std::sort(profiles->begin(), profiles->end(), [](const std::shared_ptr<stream_profile_interface>& ap,
                                         const std::shared_ptr<stream_profile_interface>& bp)
        {
            auto a = to_profile(ap.get());
            auto b = to_profile(bp.get());

            // stream == RS2_STREAM_COLOR && format == RS2_FORMAT_RGB8 element works around the fact that Y16 gets priority over RGB8 when both
            // are available for pipeline stream resolution
            auto at = std::make_tuple(a.stream, a.width, a.height, a.fps, a.stream == RS2_STREAM_COLOR && a.format == RS2_FORMAT_RGB8, a.format);
            auto bt = std::make_tuple(b.stream, b.width, b.height, b.fps, b.stream == RS2_STREAM_COLOR && b.format == RS2_FORMAT_RGB8, b.format);

            return at > bt;
        });
    }

    std::shared_ptr<stream_profile_interface> clone_profile(std::shared_ptr<stream_profile_interface> profile)
    {
        auto cloned = profile->clone();

        auto vsp = std::dynamic_pointer_cast<video_stream_profile>(cloned);
        if (vsp)
        {
            vsp->set_dims(vsp->get_width(), vsp->get_height());
        }
        cloned->set_unique_id(profile->get_unique_id());
        cloned->set_format(profile->get_format());
        cloned->set_stream_index(profile->get_stream_index());
        cloned->set_stream_type(profile->get_stream_type());
        cloned->set_framerate(profile->get_framerate());

        return cloned;
    }

    bool synthetic_sensor::is_duplicated_profile(std::shared_ptr<stream_profile_interface> duplicate, stream_profiles profiles)
    {
        auto dup_iter = std::find_if(profiles.begin(), profiles.end(), [&duplicate](std::shared_ptr<stream_profile_interface> spi)
        {
            auto sp = std::dynamic_pointer_cast<video_stream_profile>(spi);
            auto cp = std::dynamic_pointer_cast<video_stream_profile>(duplicate);
            bool res = true;

            if (sp && cp)
                res = sp->get_height() == cp->get_height() &&
                sp->get_width() == cp->get_width();

            return (res &&
                spi->get_format() == duplicate->get_format() &&
                //spi->get_unique_id() == p->get_unique_id() &&
                spi->get_stream_index() == duplicate->get_stream_index() &&
                spi->get_stream_type() == duplicate->get_stream_type() &&
                spi->get_framerate() == duplicate->get_framerate());
        });

        return dup_iter != profiles.end();
    }

    stream_profiles synthetic_sensor::init_stream_profiles()
    {
        stream_profiles result_profiles;
        auto profiles = _raw_sensor->get_stream_profiles();

        // motion profiles are passed-through
        if (Is<hid_sensor, sensor_base>(_raw_sensor))
        {
            for (auto profile : profiles)
            {
                auto target = stream_info(profile);
                _source_to_target_profiles_map[profile].push_back(profile);
                _target_to_source_profiles_map[target].push_back(profile);

            }
            return profiles;
        }

        // handle non-motion profiles
        for (auto&& pbf : _pb_factories)
        {
            auto& sources = pbf.get_source_info();
            auto& targets = pbf.get_target_info();

            for (auto& source : sources)
            {
                // add profiles that are supported by the device
                for (auto& profile : profiles)
                {
                    auto vsp = std::dynamic_pointer_cast<video_stream_profile>(profile);
                    if (profile->get_format() == source.format)
                    {
                        for (auto&& target : targets)
                        {
                            target.fps = profile->get_framerate();

                            auto cloned_profile = clone_profile(profile);
                            cloned_profile->set_format(target.format);
                            cloned_profile->set_stream_index(target.index);
                            cloned_profile->set_stream_type(target.stream);

                            auto cloned_vsp = As<video_stream_profile, stream_profile_interface>(cloned_profile);
                            if (cloned_vsp)
                            {
                                auto res = target.stream_resolution({ vsp->get_width(), vsp->get_height() });
                                target.height = res.height;
                                target.width = res.width;
                                cloned_vsp->set_dims(target.width, target.height);
                            }

                            // cache the source to target mapping
                            //if (profile->get_format() == cloned_profile->get_format())
                            if (profile->get_stream_type() == cloned_profile->get_stream_type())
                            {
                                _source_to_target_profiles_map[profile].push_back(cloned_profile);
                            }

                            // cache each target profile to its source profiles which were generated from.
                            _target_to_source_profiles_map[target].push_back(profile);

                            // disregard duplicated from profiles list
                            if (is_duplicated_profile(cloned_profile, result_profiles))
                                continue;

                            result_profiles.push_back(cloned_profile);
                        }
                    }
                }
            }
        }

        _owner->tag_profiles(result_profiles);
        sort_profiles(&result_profiles);
        return result_profiles;
    }

    std::pair<processing_block_factory, stream_profiles> synthetic_sensor::find_requests_best_match(stream_profiles requests)
    {        
        // for video stream, the best fitting processing block is defined as the processing block which its sources
        // covers the maximum amount of requests.
        stream_profiles best_match_requests;
        processing_block_factory best_match_processing_block_factory;

        int max_satisfied_req = 0;
        int best_source_size = 0;
        int count = 0;
        for (auto&& pbf : _pb_factories)
        {
            auto satisfied_req = pbf.find_satisfied_requests(requests);
            count = satisfied_req.size();
            if (count > max_satisfied_req
                || (count == max_satisfied_req
                    && pbf.get_source_info().size() < best_source_size))
            {
                max_satisfied_req = count;
                best_source_size = pbf.get_source_info().size();
                best_match_processing_block_factory = pbf;
                best_match_requests = satisfied_req;
            }
        }

        return {best_match_processing_block_factory, best_match_requests};
    }

    std::unordered_set<std::shared_ptr<stream_profile_interface>> synthetic_sensor::map_requests_to_source_profiles(stream_profiles requests)
    {
        std::unordered_set<std::shared_ptr<stream_profile_interface>> mapped_source_profiles;
        for (auto req : requests)
        {
            stream_info output_info(req);
            auto mapped_profiles = _target_to_source_profiles_map[output_info];
            for (auto&& map : mapped_profiles)
                map->set_stream_index(req->get_stream_index());
            mapped_source_profiles.insert(begin(mapped_profiles), end(mapped_profiles));
        }

        return mapped_source_profiles;
    }

    std::vector<stream_info> convert_to_stream_info(stream_profiles profiles)
    {
        std::vector<stream_info> res;
        for (auto profile : profiles)
        {
            res.push_back(stream_info(profile));
        }
        return res;
    }

    std::shared_ptr<stream_profile_interface> synthetic_sensor::correlate_target_source_profiles(std::shared_ptr<stream_profile_interface> source_profile, std::shared_ptr<stream_profile_interface> request)
    {
        // might be one profile to many, for instance, y8i -> y8left, y8right.
        auto target_profiles = _source_to_target_profiles_map[source_profile];

        // find the closest target profile to the request, if there is none, just take the first.
        auto best_match = std::find_if(target_profiles.begin(), target_profiles.end(), [request](auto sp)
        {
            return request->get_format() == sp->get_format() &&
                request->get_stream_index() == sp->get_stream_index() &&
                request->get_stream_type() == sp->get_stream_type();
        });

        auto correlated_target_profile = best_match != target_profiles.end() ? *best_match : target_profiles.front();
        source_profile->set_stream_index(correlated_target_profile->get_stream_index());
        source_profile->set_unique_id(correlated_target_profile->get_unique_id());
        auto vsp = As<video_stream_profile, stream_profile_interface>(source_profile);
        auto cvsp = As<video_stream_profile, stream_profile_interface>(correlated_target_profile);
        if (vsp)
        {
            vsp->set_intrinsics([cvsp]() {

                if (cvsp)
                    return cvsp->get_intrinsics();
                else
                    return rs2_intrinsics{};
            });
            vsp->set_dims(cvsp->get_width(), cvsp->get_height());
        }
        return source_profile;
    }

    stream_profiles synthetic_sensor::resolve_requests(const stream_profiles& requests)
    {
        std::unordered_set<std::shared_ptr<stream_profile_interface>> resolved_req_set;
        stream_profiles resolved_req;
        stream_profiles unhandled_reqs(requests);
        
        // cache the requests
        for (auto req : requests)
        {
            cached_requests[req->get_format()].push_back(req);
        }
        
        // while not finished handling all of the requests do
        while (!unhandled_reqs.empty())
        {
            // find the best fitting processing block - the one which resolves the most requests.
            auto best_match = find_requests_best_match(unhandled_reqs);
            auto best_pb = best_match.first;
            auto best_reqs = best_match.second;
            
            // mark as handled resolved requests
            for (auto req : best_reqs)
            {
                auto unhandled_req = std::find_if(unhandled_reqs.begin(), unhandled_reqs.end(), [&req](auto sp) {
                    return stream_info(req) == stream_info(sp);
                });
                if (unhandled_req != end(unhandled_reqs))
                    unhandled_reqs.erase(unhandled_req);                   
            }

            // retrieve source profile from cached map.
            for (auto req : best_reqs)
            {
                auto target = stream_info(req);
                auto mapped_source_profiles = _target_to_source_profiles_map[target];
                for (auto source_profile : mapped_source_profiles)
                {
                    if (best_pb.has_source(source_profile))
                    {
                        // init_stream_profiles() cloned the source profiles and converted them into target profiles.
                        // we must pass the missing data from the target profiles to the source profiles.
                        resolved_req_set.insert(correlate_target_source_profiles(source_profile, req));
                    }
                }
            }
            // generate this processing block and append it to the cached processing blocks.
            _formats_to_processing_block[best_pb.get_source_info()] = best_pb.generate_processing_block();
        }
        
        resolved_req = { resolved_req_set.begin(), resolved_req_set.end() };
        return resolved_req;
    }

    void synthetic_sensor::open(const stream_profiles& requests)
    {
        std::lock_guard<std::mutex> lock(_configure_lock);
        auto resolved_req = resolve_requests(requests);

        _raw_sensor->set_owner_sensor(this->shared_from_this()); // TODO - Ariel - maybe move set_owner_sensor method to sensor_base if it is common for hid and uvc
        _raw_sensor->open(resolved_req);
    }

    void synthetic_sensor::close()
    {
        std::lock_guard<std::mutex> lock(_configure_lock);
        _raw_sensor->close();
        _formats_to_processing_block.erase(begin(_formats_to_processing_block), end(_formats_to_processing_block));
    }

    template<class T>
    frame_callback_ptr make_callback(T callback)
    {
        return {
            new internal_frame_callback<T>(callback),
            [](rs2_frame_callback* p) { /*p->release(); */}
        };
    }

    std::shared_ptr<stream_profile_interface> synthetic_sensor::filter_frame_by_requests(frame_interface* f)
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

        // After processing callback
        auto output_cb = make_callback([&, callback](frame_holder f) {
            std::vector<frame_interface*> frames_to_process;
            frames_to_process.push_back(f.frame);

            auto composite = dynamic_cast<composite_frame*>(f.frame);
            if (composite)
            {
                for (int i = 0; i < composite->get_embedded_frames_count(); i++)
                {
                    frames_to_process.push_back(composite->get_frame(i));
                }
            }

            // process only frames which aren't composite.
            for (auto fr : frames_to_process)
            {
                if (!dynamic_cast<composite_frame*>(fr))
                {
                    auto cached_profile = filter_frame_by_requests(fr);

                    if (cached_profile)
                    {
                        fr->set_stream(cached_profile);
                    }
                    else
                        continue;

                    fr->acquire();
                    callback->on_frame((rs2_frame*)fr);
                }

            }
        });

        // Set callbacks for all of the relevant processing blocks
        for (auto&& pb_entry : _formats_to_processing_block)
        {
            auto&& pb = pb_entry.second;
            if (pb)
            {
                pb_entry.second->set_output_callback(output_cb);
            }
        }

        // Invoke processing blocks callback
        auto process_cb = make_callback([&, callback, this](frame_holder f) {
            //std::lock_guard<std::mutex> lock(_configure_lock); // TODO - Ariel - maybe can cause deadlock?
            for (auto&& pb_entry : _formats_to_processing_block)
            {
                if (!f)
                    return;
                // TODO - Ariel - handle multiple requests
                auto&& pb = pb_entry.second;
                auto&& sources_info = pb_entry.first;

                // process only if the frame format matches the processing block source format
                auto sp = f->get_stream();
                auto source_info_it = std::find_if(sources_info.begin(), sources_info.end(), [&f, &sp](auto info)
                {
                    return sp->get_format() == info.format;
                });
                if (source_info_it == sources_info.end())
                    continue;

                pb->invoke(std::move(f));
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

    void synthetic_sensor::register_processing_block(std::vector<stream_info> from, std::vector<stream_info> to, std::function<std::shared_ptr<processing_block>(void)> generate_func)
    {
        processing_block_factory pbf(from, to, generate_func);
        _pb_factories.push_back(pbf);
    }

    processing_block_factory::processing_block_factory(std::vector<stream_info> from, std::vector<stream_info> to, std::function<std::shared_ptr<processing_block>(void)> generate_func) :
        _source_info(from), _target_info(to), generate_processing_block(generate_func)
    {}

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

        auto&& rhs_src_fmts = rhs.get_source_info();
        for (auto src : _source_info)
        {
            if (std::find_if(rhs_src_fmts.begin(), rhs_src_fmts.end(), [&src](auto fmt) {
                return src == fmt;
            }) == rhs_src_fmts.end())
                return false;
        }

        auto&& rhs_tgt_fmts = rhs.get_target_info();
        for (auto tgt : _target_info)
        {
            if (std::find_if(rhs_tgt_fmts.begin(), rhs_tgt_fmts.end(), [&tgt](auto rhs_tgt) {
                return tgt == rhs_tgt;
            }) == rhs_tgt_fmts.end())
                return false;
        }

        return true;
    }

    bool processing_block_factory::has_source(std::shared_ptr<stream_profile_interface> source)
    {
        for (auto s : _source_info)
        {
            if (source->get_format() == s.format)
                return true;
        }
        return false;
    }

    stream_profiles processing_block_factory::find_satisfied_requests(stream_profiles requests)
    {
        // Return all requests which satisfies the processing block.
        // a processing block is satisfied, if ALL of its sources found a match with a request.

        stream_profiles satisfied_req;
        auto tgts = _target_info;
        for (auto&& req : requests)
        {
            // for motion profile, request is satisfied if it has the same format and stream type.
            if (Is<motion_stream_profile, stream_profile_interface>(req))
            {
                if (std::find_if(begin(tgts), end(tgts), [&req](auto tgt) {
                    return req->get_format() == tgt.format &&
                        req->get_stream_type() == tgt.stream;
                }) != end(tgts))
                    satisfied_req.push_back(req);
            }
            else
            {
                if (std::find_if(begin(tgts), end(tgts), [&req](auto tgt) {
                    return req->get_format() == tgt.format;
                }) != end(tgts))
                    satisfied_req.push_back(req);
            }
        }
        return satisfied_req;
    }

    void processing_block_factory::copy_processing_block_factory(const processing_block_factory & rhs)
    {
        _source_info = rhs.get_source_info();
        _target_info = rhs.get_target_info();
        generate_processing_block = rhs.generate_processing_block;
    }

    stream_info::stream_info(rs2_format fmt, rs2_stream strm, int idx, uint32_t w, uint32_t h, int framerate, resolution_func res_func) :
        format(fmt), stream(strm), index(idx), height(h), width(w), stream_resolution(res_func), fps(framerate)
    {
        auto res = stream_resolution({ w, h });
        width = res.width;
        height = res.height;
    }

    stream_info::stream_info(std::shared_ptr<stream_profile_interface> sp)
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

    stream_info::stream_info(const stream_info & other)
    {
        copy(other);
    }

    bool stream_info::operator==(const stream_info & rhs)
    {
        return 
            rhs.width == width && rhs.height == height &&
            rhs.format == format &&
            rhs.stream == stream &&
            rhs.fps == fps &&
            rhs.index == index;
    }

    void stream_info::copy(const stream_info & other)
    {
        stream_resolution = other.stream_resolution;
        format = other.format;
        stream = other.stream;
        width = other.width;
        height = other.height;
        index = other.index;
        fps = other.fps;
    }

    stream_info & stream_info::operator=(const stream_info & rhs)
    {
        if (this == &rhs)
            return *this;

        copy(rhs);
        return *this;
    }

    bool operator<(const stream_info & lhs, const stream_info & rhs)
    {
        return (lhs.format < rhs.format ||
            lhs.index < rhs.index && lhs.format != rhs.format);
    }
    bool operator==(const stream_info & lhs, const stream_info & rhs)
    {
        return rhs.format == lhs.format &&
            rhs.stream == lhs.stream &&
            rhs.width == lhs.width &&
            rhs.height == lhs.height &&
            rhs.fps == lhs.fps &&
            rhs.index == lhs.index;
    }
}
