// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2018 Intel Corporation. All Rights Reserved.

#include <vector>
#include "l500-device.h"
#include "context.h"
#include "stream.h"
#include "l500-depth.h"
#include "l500-private.h"
#include "proc/decimation-filter.h"
#include "proc/threshold.h" 
#include "proc/spatial-filter.h"
#include "proc/temporal-filter.h"
#include "proc/hole-filling-filter.h"
#include "proc/zero-order.h"
#include "proc/syncer-processing-block.h"
#include "proc/identity-processing-block.h"
#include "proc/l500-transform.h"

namespace librealsense
{
    using namespace ivcam2;

    l500_device::l500_device(std::shared_ptr<context> ctx,
        const platform::backend_device_group& group)
        :device(ctx, group), global_time_interface(), 
        _depth_stream(new stream(RS2_STREAM_DEPTH)),
        _ir_stream(new stream(RS2_STREAM_INFRARED)),
        _confidence_stream(new stream(RS2_STREAM_CONFIDENCE))
    {
        _depth_device_idx = add_sensor(create_depth_device(ctx, group.uvc_devices));
        auto pid = group.uvc_devices.front().pid;
        std::string device_name = (rs500_sku_names.end() != rs500_sku_names.find(pid)) ? rs500_sku_names.at(pid) : "RS5xx";

        using namespace ivcam2;

        auto&& backend = ctx->get_backend();

        auto& depth_sensor = get_depth_sensor();
        auto& raw_depth_sensor = get_raw_depth_sensor();

        if (group.usb_devices.size() > 0)
        {
            _hw_monitor = std::make_shared<hw_monitor>(
                std::make_shared<locked_transfer>(backend.create_usb_device(group.usb_devices.front()),
                    raw_depth_sensor));
        }
        else
        {
            _hw_monitor = std::make_shared<hw_monitor>(
                std::make_shared<locked_transfer>(std::make_shared<command_transfer_over_xu>(
                    raw_depth_sensor, depth_xu, L500_HWMONITOR),
                    raw_depth_sensor));
        }

#ifdef HWM_OVER_XU
        if (group.usb_devices.size() > 0)
        {
            _hw_monitor = std::make_shared<hw_monitor>(
                std::make_shared<locked_transfer>(std::make_shared<command_transfer_over_xu>(
                    raw_depth_sensor, depth_xu, L500_HWMONITOR),
                    raw_depth_sensor));
        }
#endif

        std::vector<uint8_t> gvd_buff(HW_MONITOR_BUFFER_SIZE);
        _hw_monitor->get_gvd(gvd_buff.size(), gvd_buff.data(), GVD);
        // fooling tests recordings - don't remove
        _hw_monitor->get_gvd(gvd_buff.size(), gvd_buff.data(), GVD);

        auto optic_serial = _hw_monitor->get_module_serial_string(gvd_buff, module_serial_offset, module_serial_size);
        auto asic_serial = _hw_monitor->get_module_serial_string(gvd_buff, module_asic_serial_offset, module_serial_size);
        auto fwv = _hw_monitor->get_firmware_version_string(gvd_buff, fw_version_offset);
        _fw_version = firmware_version(fwv);

        auto pid_hex_str = hexify(group.uvc_devices.front().pid);

        using namespace platform;

        auto usb_mode = raw_depth_sensor.get_usb_specification();
        if (usb_spec_names.count(usb_mode) && (usb_undefined != usb_mode))
        {
            auto usb_type_str = usb_spec_names.at(usb_mode);
            register_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR, usb_type_str);
        }

        register_info(RS2_CAMERA_INFO_NAME, device_name);
        register_info(RS2_CAMERA_INFO_SERIAL_NUMBER, optic_serial);
        register_info(RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER, asic_serial);
        register_info(RS2_CAMERA_INFO_FIRMWARE_VERSION, _fw_version);
        register_info(RS2_CAMERA_INFO_DEBUG_OP_CODE, std::to_string(static_cast<int>(fw_cmd::GLD)));
        register_info(RS2_CAMERA_INFO_PHYSICAL_PORT, group.uvc_devices.front().device_path);
        register_info(RS2_CAMERA_INFO_PRODUCT_ID, pid_hex_str);
        register_info(RS2_CAMERA_INFO_PRODUCT_LINE, "L500");
    }

    std::shared_ptr<synthetic_sensor> l500_device::create_depth_device(std::shared_ptr<context> ctx,
        const std::vector<platform::uvc_device_info>& all_device_infos)
    {
        auto&& backend = ctx->get_backend();

        std::vector<std::shared_ptr<platform::uvc_device>> depth_devices;
        for (auto&& info : filter_by_mi(all_device_infos, 0)) // Filter just mi=0, DEPTH
            depth_devices.push_back(backend.create_uvc_device(info));

        std::unique_ptr<frame_timestamp_reader> timestamp_reader_metadata(new l500_timestamp_reader_from_metadata(backend.create_time_service()));
        auto enable_global_time_option = std::shared_ptr<global_time_option>(new global_time_option());
        auto depth_ep = std::make_shared<uvc_sensor>("Depth Sensor", std::make_shared<platform::multi_pins_uvc_device>(depth_devices),
            std::unique_ptr<frame_timestamp_reader>(new global_timestamp_reader(std::move(timestamp_reader_metadata), _tf_keeper, enable_global_time_option)), this);
        depth_ep->register_xu(depth_xu);

        auto smart_depth_ep = std::make_shared<l500_depth_sensor>(this, depth_ep);
        smart_depth_ep->register_option(RS2_OPTION_GLOBAL_TIME_ENABLED, enable_global_time_option);
        smart_depth_ep->register_option(RS2_OPTION_VISUAL_PRESET,
            std::make_shared<uvc_xu_option<int>>(
                *depth_ep,
                ivcam2::depth_xu,
                ivcam2::L500_DEPTH_VISUAL_PRESET, "Preset to calibrate the camera to short or long range. 1 is long range and 2 is short range",
                std::map<float, std::string>{ { 1, "Long range"},
                { 2, "Short range" }}));

        smart_depth_ep->register_processing_block(
            { {RS2_FORMAT_Y8} },
            { {RS2_FORMAT_Y8, RS2_STREAM_INFRARED, 1, 0, 0, 0, &rotate_resolution} },
            []() {
            return std::make_shared<l500_transform>(RS2_FORMAT_Y8);
        });

        smart_depth_ep->register_processing_block(
            { {RS2_FORMAT_RAW8} },
            { {RS2_FORMAT_RAW8, RS2_STREAM_CONFIDENCE, 0, 0, 0, 0, &l500_confidence_resolution} },
            []() {
            return std::make_shared<l500_transform>(RS2_FORMAT_RAW8);
        });

        smart_depth_ep->register_processing_block(
            { {RS2_FORMAT_Z16}, {RS2_FORMAT_Y8} },
            { {RS2_FORMAT_Z16, RS2_STREAM_DEPTH, 0, 0, 0, 0, &rotate_resolution} },
            []() {
            auto z16rot = std::make_shared<l500_transform>(RS2_FORMAT_Z16);
            auto y8rot = std::make_shared<l500_transform>(RS2_FORMAT_Y8);
            auto sync = std::make_shared<syncer_process_unit>();
            auto zo = std::make_shared<zero_order>();

            auto cpb = std::make_shared<composite_processing_block>();
            cpb->add(z16rot);
            cpb->add(y8rot);
            cpb->add(sync);
            cpb->add(zo);

            return cpb;
        });

        smart_depth_ep->register_processing_block(
            { {RS2_FORMAT_Z16}, {RS2_FORMAT_Y8}, {RS2_FORMAT_RAW8} },
            {
                {RS2_FORMAT_Z16, RS2_STREAM_DEPTH, 0, 0, 0, 0, &rotate_resolution},
                {RS2_FORMAT_RAW8, RS2_STREAM_CONFIDENCE, 0, 0, 0, 0, &l500_confidence_resolution}
            },
            []() {
            auto z16rot = std::make_shared<l500_transform>(RS2_FORMAT_Z16);
            auto y8rot = std::make_shared<l500_transform>(RS2_FORMAT_Y8);
            auto conf = std::make_shared<l500_transform>(RS2_FORMAT_RAW8);
            auto sync = std::make_shared<syncer_process_unit>();
            auto zo = std::make_shared<zero_order>();

            auto cpb = std::make_shared<composite_processing_block>();
            cpb->add(z16rot);
            cpb->add(y8rot);
            cpb->add(conf);
            cpb->add(sync);
            cpb->add(zo);

            return cpb;
        });

        return smart_depth_ep;
    }

    void l500_device::force_hardware_reset() const
    {
        command cmd(ivcam2::fw_cmd::HW_RESET);
        cmd.require_response = false;
        _hw_monitor->send(cmd);
    }

    void l500_device::create_snapshot(std::shared_ptr<debug_interface>& snapshot) const
    {
        throw not_implemented_exception("create_snapshot(...) not implemented!");
    }

    void l500_device::enable_recording(std::function<void(const debug_interface&)> record_action)
    {
        throw not_implemented_exception("enable_recording(...) not implemented!");
    }

    double l500_device::get_device_time_ms()
    {
        if (dynamic_cast<const platform::playback_backend*>(&(get_context()->get_backend())) != nullptr)
        {
            throw not_implemented_exception("device time not supported for backend.");
        }

        if (!_hw_monitor)
            throw wrong_api_call_sequence_exception("_hw_monitor is not initialized yet");

        command cmd(ivcam2::fw_cmd::MRD, ivcam2::REGISTER_CLOCK_0, ivcam2::REGISTER_CLOCK_0 + 4);
        auto res = _hw_monitor->send(cmd);

        if (res.size() < sizeof(uint32_t))
        {
            LOG_DEBUG("size(res):" << res.size());
            throw std::runtime_error("Not enough bytes returned from the firmware!");
        }
        uint32_t dt = *(uint32_t*)res.data();
        double ts = dt * TIMESTAMP_USEC_TO_MSEC;
        return ts;
    }

    notification l500_notification_decoder::decode(int value)
    {
        if (l500_fw_error_report.find(static_cast<uint8_t>(value)) != l500_fw_error_report.end())
            return{ RS2_NOTIFICATION_CATEGORY_HARDWARE_ERROR, value, RS2_LOG_SEVERITY_ERROR, l500_fw_error_report.at(static_cast<uint8_t>(value)) };

        return{ RS2_NOTIFICATION_CATEGORY_HARDWARE_ERROR, value, RS2_LOG_SEVERITY_WARN, (to_string() << "L500 HW report - unresolved type " << value) };
    }
}
