// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#pragma once

#include "align.h"
#include "types.h"

namespace librealsense
{
    std::shared_ptr<librealsense::align> create_align(rs2_stream align_to);

    class processing_block_factory
    {
    public:
        processing_block_factory() {};

        processing_block_factory(std::vector<stream_profile> from,
            std::vector<stream_profile> to,
            std::function<std::shared_ptr<processing_block>(void)> generate_func);

        std::function<std::shared_ptr<processing_block>(void)> generate_processing_block;

        std::vector<stream_profile> get_source_info() const { return _source_info; }
        std::vector<stream_profile> get_target_info() const { return _target_info; }

        bool operator==(const processing_block_factory& rhs);

        stream_profiles find_satisfied_requests(stream_profiles sp);
        bool has_source(std::shared_ptr<stream_profile_interface> source);

    protected:
        std::vector<stream_profile> _source_info;
        std::vector<stream_profile> _target_info;
    };
}
