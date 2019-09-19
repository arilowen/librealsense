// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include "processing-blocks-factory.h"
#include "sse/sse-align.h"
#include "cuda/cuda-align.h"

#include "stream.h"

namespace librealsense
{
#ifdef RS2_USE_CUDA
    std::shared_ptr<librealsense::align> create_align(rs2_stream align_to)
    {
        return std::make_shared<librealsense::align_cuda>(align_to);
    }
#else
#ifdef __SSSE3__
    std::shared_ptr<librealsense::align> create_align(rs2_stream align_to)
    {
        return std::make_shared<librealsense::align_sse>(align_to);
    }
#else // No optimizations
    std::shared_ptr<librealsense::align> create_align(rs2_stream align_to)
    {
        return std::make_shared<librealsense::align>(align_to);
    }
#endif // __SSSE3__
#endif // RS2_USE_CUDA

    processing_block_factory::processing_block_factory(std::vector<stream_profile> from, std::vector<stream_profile> to, std::function<std::shared_ptr<processing_block>(void)> generate_func) :
        _source_info(from), _target_info(to), generate_processing_block(generate_func)
    {}

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
}
