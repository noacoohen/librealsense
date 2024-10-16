// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/hpp/rs_sensor.hpp>
#include <librealsense2/hpp/rs_processing.hpp>

#include <numeric>
#include <cmath>
#include "environment.h"
#include "option.h"
#include "stream.h"
#include "core/video.h"
#include "proc/synthetic-stream.h"
#include "proc/rotation-filter.h"
#include "proc/rotation-transform.h"

#include <rsutils/string/from.h>

namespace librealsense {

    const int rotation_min_val = -90;
    const int rotation_max_val = 180;  
    const int rotation_default_val = 0;
    const int rotation_step = 90; 

    rotation_filter::rotation_filter() :
        stream_filter_processing_block("Rotation Filter"),
        _decimation_factor(rotation_default_val),
        _control_val(rotation_default_val),
        _patch_size(rotation_default_val),
        _kernel_size(_patch_size*_patch_size),
        _real_width(),
        _real_height(0),
        _padded_width(0),
        _padded_height(0),
        _recalc_profile(false),
        _options_changed(false)
    {
        _stream_filter.stream = RS2_STREAM_DEPTH;
        _stream_filter.format = RS2_FORMAT_Z16;

        auto rotation_control = std::make_shared< ptr_option< int > >(
            rotation_min_val,
            rotation_max_val,
            rotation_step,
            rotation_default_val,
            &_control_val, "Rotation scale");

        auto weak_rotation_control = std::weak_ptr< ptr_option< int > >( rotation_control );
        rotation_control->on_set(
            [this, weak_rotation_control]( float val )
        {
            auto strong_rotation_control = weak_rotation_control.lock();
            if(!strong_rotation_control) return;

            std::lock_guard<std::mutex> lock(_mutex);

            if( ! strong_rotation_control->is_valid( val ) )
                throw invalid_value_exception( rsutils::string::from()
                                               << "Unsupported rotation scale " << val << " is out of range." );

            _value = _control_val; 
            
        });

        register_option( RS2_OPTION_ROTATION, rotation_control );
    }

    rs2::frame rotation_filter::process_frame(const rs2::frame_source& source, const rs2::frame& f)
    {

        auto src = f.as<rs2::video_frame>();
        rs2::stream_profile profile = f.get_profile();
        _target_stream_profile = profile;
        rs2_format format = profile.format();
        _padded_width = src.get_height();
        _padded_height = src.get_width();
        rs2_stream type = profile.stream_type();
        auto bpp = src.get_bytes_per_pixel();

        rs2_extension tgt_type;
        if (type == RS2_STREAM_COLOR || type == RS2_STREAM_INFRARED)
            tgt_type = RS2_EXTENSION_VIDEO_FRAME;
        else
            tgt_type = f.is<rs2::disparity_frame>() ? RS2_EXTENSION_DISPARITY_FRAME : RS2_EXTENSION_DEPTH_FRAME;

        if (auto tgt = prepare_target_frame(f, source, tgt_type))
        {
            int rotated_width = src.get_width();
            int rotated_height = src.get_height();
            switch( bpp )
            {
            case 1:
                rotate_depth< 1 >( static_cast< const uint8_t * >( src.get_data() ),
                                   static_cast< uint8_t * >( const_cast< void * >( tgt.get_data() ) ),
                                   
                                             rotated_width,
                                             rotated_height);
                break;
            case 2:
                rotate_depth< 2 >( static_cast< const uint8_t * >( src.get_data() ),
                                   static_cast< uint8_t * >( const_cast< void * >( tgt.get_data() ) ),
                                             rotated_width,
                                             rotated_height);
                break;
            default:
                LOG_ERROR( "Rotation transform does not support format: "
                           + std::string( rs2_format_to_string( tgt.get_profile().format() ) ) );
            }
            /*if (format == RS2_FORMAT_Z16)
            {
                rotate_depth(static_cast<const uint8_t*>(src.get_data()),
                    static_cast<uint8_t*>(const_cast<void*>(tgt.get_data())),
                    src.get_width(), src.get_height(), this->_patch_size);
            }
            else
            {
                rotate_others(format, src.get_data(),
                    const_cast<void*>(tgt.get_data()),
                    src.get_width(), src.get_height(), this->_patch_size);
            }
            return tgt;*/
        }
        return f;
    }

    void  rotation_filter::update_output_profile(const rs2::frame& f)
    {
        if (_options_changed || f.get_profile().get() != _source_stream_profile.get())
        {
            _options_changed = false;
            _source_stream_profile = f.get_profile();
            const auto pf = _registered_profiles.find(std::make_tuple(_source_stream_profile.get(), _decimation_factor));
            if (_registered_profiles.end() != pf)
            {
                _target_stream_profile = pf->second;
                auto tgt_vspi = dynamic_cast<video_stream_profile_interface*>(_target_stream_profile.get()->profile);
                if (!tgt_vspi)
                    throw std::runtime_error("Stream profile interface is not video stream profile interface");

                auto f_pf = dynamic_cast<video_stream_profile_interface*>(_source_stream_profile.get()->profile);
                if (!f_pf)
                    throw std::runtime_error("Stream profile interface is not video stream profile interface");

                rs2_intrinsics tgt_intrin = tgt_vspi->get_intrinsics();

                // Update real/padded output frame size based on retrieved input properties
                _real_width = f_pf->get_width() / _patch_size;
                _real_height = f_pf->get_height() / _patch_size;
                _padded_width = tgt_intrin.width;
                _padded_height = tgt_intrin.height;
            }
            else
            {
                _recalc_profile = true;
            }
        }

        // Buld a new target profile for every system/filter change
        if (_recalc_profile)
        {
            auto vp = _source_stream_profile.as<rs2::video_stream_profile>();

            auto tmp_profile = _source_stream_profile.clone(_source_stream_profile.stream_type(), _source_stream_profile.stream_index(), _source_stream_profile.format());
            auto src_vspi = dynamic_cast<video_stream_profile_interface*>(_source_stream_profile.get()->profile);
            if (!src_vspi)
                throw std::runtime_error("Stream profile interface is not video stream profile interface");

            auto tgt_vspi = dynamic_cast<video_stream_profile_interface*>(tmp_profile.get()->profile);
            if( ! tgt_vspi )
                throw std::runtime_error( "Profile is not video stream profile" );

            rs2_intrinsics src_intrin = src_vspi->get_intrinsics();
            rs2_intrinsics tgt_intrin = tgt_vspi->get_intrinsics();

            // recalculate real/padded output frame size based on new input porperties
            _real_width = src_vspi->get_width() / _patch_size;
            _real_height = src_vspi->get_height() / _patch_size;

            // The resulted frame dimension will be dividible by 4;
            _padded_width = _real_width + 3;
            _padded_width /= 4;
            _padded_width *= 4;

            _padded_height = _real_height + 3;
            _padded_height /= 4;
            _padded_height *= 4;

            tgt_intrin.width = _padded_width;
            tgt_intrin.height = _padded_height;
            tgt_intrin.fx = src_intrin.fx / _patch_size;
            tgt_intrin.fy = src_intrin.fy / _patch_size;
            tgt_intrin.ppx = src_intrin.ppx / _patch_size;
            tgt_intrin.ppy = src_intrin.ppy / _patch_size;

            tgt_vspi->set_intrinsics([tgt_intrin]() { return tgt_intrin; });
            tgt_vspi->set_dims(tgt_intrin.width, tgt_intrin.height);

            _registered_profiles[std::make_tuple(_source_stream_profile.get(), _decimation_factor)] = _target_stream_profile = tmp_profile;

            _recalc_profile = false;
        }
    }

    rs2::frame rotation_filter::prepare_target_frame(const rs2::frame& f, const rs2::frame_source& source, rs2_extension tgt_type)
    {
        auto vf = f.as<rs2::video_frame>();
        auto ret = source.allocate_video_frame(_target_stream_profile, f,
            vf.get_bytes_per_pixel(),
            _padded_height,
            _padded_width,
            _padded_height * vf.get_bytes_per_pixel(),
            tgt_type);

        return ret;
    }

    template< size_t SIZE >
    void rotation_filter::rotate_depth( const uint8_t * frame_data_in,  uint8_t * const frame_data_out,
        size_t width, size_t height)
    {
        auto width_out = height;
        auto height_out = width;

        uint8_t buffer[8][8 * SIZE];  // = { 0 };
        for( int i = 0; i <= height - 8; i = i + 8 )
        {
            for( int j = 0; j <= width - 8; j = j + 8 )
            {
                for( int ii = 0; ii < 8; ++ii )
                {
                    for( int jj = 0; jj < 8; ++jj )
                    {
                        auto source_index = ( ( j + jj ) + ( width * ( i + ii ) ) ) * SIZE;
                        memcpy( (void *)( &buffer[7 - jj][( 7 - ii ) * SIZE] ), &frame_data_in[source_index], SIZE );
                    }
                }

                for( int ii = 0; ii < 8; ++ii )
                {
                    auto out_index = ( ( ( height_out - 8 - j + 1 ) * width_out ) - i - 8 + (ii)*width_out );
                    memcpy( &frame_data_out[(out_index)*SIZE], &( buffer[ii] ), 8 * SIZE );
                }
            }
        }
        
    }

    void rotation_filter::rotate_others(rs2_format format, const void * frame_data_in, void * frame_data_out,
        size_t width_in, size_t height_in, size_t scale)
    {
        int sum = 0;
        auto patch_size = scale * scale;

        switch (format)
        {
        case RS2_FORMAT_YUYV:
        {
            uint8_t* from = (uint8_t*)frame_data_in;
            uint8_t* p = nullptr;
            uint8_t* q = (uint8_t*)frame_data_out;

            auto w_2 = width_in >> 1;
            auto rw_2 = _real_width >> 1;
            auto pw_2 = _padded_width >> 1;
            auto s2 = scale >> 1;
            bool odd = (scale & 1);
            for (int j = 0; j < _real_height; ++j)
            {
                for (int i = 0; i < rw_2; ++i)
                {
                    p = from + scale * (j * w_2 + i) * 4;
                    sum = 0;
                    for (size_t n = 0; n < scale; ++n)
                    {
                        for (size_t m = 0; m < scale; ++m)
                            sum += p[m * 2];

                        p += w_2 * 4;
                    }
                    *q++ = (uint8_t)(sum / patch_size);

                    p = from + scale * (j * w_2 + i) * 4 + 1;
                    sum = 0;
                    for (size_t n = 0; n < scale; ++n)
                    {
                        for (size_t m = 0; m < s2; ++m)
                            sum += 2 * p[m * 4];

                        if (odd)
                            sum += p[s2 * 4];

                        p += w_2 * 4;
                    }
                    *q++ = (uint8_t)(sum / patch_size);

                    p = from + scale * (j * w_2 + i) * 4 + s2 * 4 + (odd ? 2 : 0);
                    sum = 0;
                    for (size_t n = 0; n < scale; ++n)
                    {
                        for (size_t m = 0; m < scale; ++m)
                            sum += p[m * 2];

                        p += w_2 * 4;
                    }
                    *q++ = (uint8_t)(sum / patch_size);

                    p = from + scale * (j * w_2 + i) * 4 + 3;
                    sum = 0;
                    for (size_t n = 0; n < scale; ++n)
                    {
                        for (size_t m = 0; m < s2; ++m)
                            sum += 2 * p[m * 4];

                        if (odd)
                            sum += p[s2 * 4];

                        p += w_2 * 4;
                    }
                    *q++ = (uint8_t)(sum / patch_size);
                }

                for (int i = rw_2; i < pw_2; ++i)
                {
                    *q++ = 0;
                    *q++ = 0;
                    *q++ = 0;
                    *q++ = 0;
                }
            }

            for (int j = _real_height; j < _padded_height; ++j)
            {
                for (int i = 0; i < _padded_width; ++i)
                {
                    *q++ = 0;
                    *q++ = 0;
                }
            }
        }
        break;

        case RS2_FORMAT_UYVY:
        {
            uint8_t* from = (uint8_t*)frame_data_in;
            uint8_t* p = nullptr;
            uint8_t* q = (uint8_t*)frame_data_out;

            auto w_2 = width_in >> 1;
            auto rw_2 = _real_width >> 1;
            auto pw_2 = _padded_width >> 1;
            auto s2 = scale >> 1;
            bool odd = (scale & 1);
            for (int j = 0; j < _real_height; ++j)
            {
                for (int i = 0; i < rw_2; ++i)
                {
                    p = from + scale * (j * w_2 + i) * 4;
                    sum = 0;
                    for (size_t n = 0; n < scale; ++n)
                    {
                        for (size_t m = 0; m < s2; ++m)
                            sum += 2 * p[m * 4];

                        if (odd)
                            sum += p[s2 * 4];

                        p += w_2 * 4;
                    }
                    *q++ = (uint8_t)(sum / patch_size);

                    p = from + scale * (j * w_2 + i) * 4 + 1;
                    sum = 0;
                    for (size_t n = 0; n < scale; ++n)
                    {
                        for (size_t m = 0; m < scale; ++m)
                            sum += p[m * 2];

                        p += w_2 * 4;
                    }
                    *q++ = (uint8_t)(sum / patch_size);

                    p = from + scale * (j * w_2 + i) * 4 + 2;
                    sum = 0;
                    for (size_t n = 0; n < scale; ++n)
                    {
                        for (size_t m = 0; m < s2; ++m)
                            sum += 2 * p[m * 4];

                        if (odd)
                            sum += p[s2 * 4];

                        p += w_2 * 4;
                    }
                    *q++ = (uint8_t)(sum / patch_size);

                    p = from + scale * (j * w_2 + i) * 4 + s2 * 4 + (odd ? 3 : 1);
                    sum = 0;
                    for (size_t n = 0; n < scale; ++n)
                    {
                        for (size_t m = 0; m < scale; ++m)
                            sum += p[m * 2];

                        p += w_2 * 4;
                    }
                    *q++ = (uint8_t)(sum / patch_size);
                }

                for (int i = rw_2; i < pw_2; ++i)
                {
                    *q++ = 0;
                    *q++ = 0;
                    *q++ = 0;
                    *q++ = 0;
                }
            }

            for (int j = _real_height; j < _padded_height; ++j)
            {
                for (int i = 0; i < _padded_width; ++i)
                {
                    *q++ = 0;
                    *q++ = 0;
                }
            }
        }
        break;

        case RS2_FORMAT_RGB8:
        case RS2_FORMAT_BGR8:
        {
            uint8_t* from = (uint8_t*)frame_data_in;
            uint8_t* p = nullptr;
            uint8_t* q = (uint8_t*)frame_data_out;;

            for (int j = 0; j < _real_height; ++j)
            {
                for (int i = 0; i < _real_width; ++i)
                {
                    for (int k = 0; k < 3; ++k)
                    {
                        p = from + scale * (j * width_in + i) * 3 + k;
                        sum = 0;
                        for (size_t n = 0; n < scale; ++n)
                        {
                            for (size_t m = 0; m < scale; ++m)
                                sum += p[m * 3];

                            p += width_in * 3;
                        }

                        *q++ = (uint8_t)(sum / patch_size);
                    }
                }

                for (int i = _real_width; i < _padded_width; ++i)
                {
                    *q++ = 0;
                    *q++ = 0;
                    *q++ = 0;
                }
            }

            for (int j = _real_height; j < _padded_height; ++j)
            {
                for (int i = 0; i < _padded_width; ++i)
                {
                    *q++ = 0;
                    *q++ = 0;
                    *q++ = 0;
                }
            }
        }
        break;

        case RS2_FORMAT_RGBA8:
        case RS2_FORMAT_BGRA8:
        {
            uint8_t* from = (uint8_t*)frame_data_in;
            uint8_t* p = nullptr;
            uint8_t* q = (uint8_t*)frame_data_out;

            for (int j = 0; j < _real_height; ++j)
            {
                for (int i = 0; i < _real_width; ++i)
                {
                    for (int k = 0; k < 4; ++k)
                    {
                        p = from + scale * (j * width_in + i) * 4 + k;
                        sum = 0;
                        for (size_t n = 0; n < scale; ++n)
                        {
                            for (size_t m = 0; m < scale; ++m)
                                sum += p[m * 4];

                            p += width_in * 4;
                        }

                        *q++ = (uint8_t)(sum / patch_size);
                    }
                }

                for (int i = _real_width; i < _padded_width; ++i)
                {
                    *q++ = 0;
                    *q++ = 0;
                    *q++ = 0;
                    *q++ = 0;
                }
            }

            for (int j = _real_height; j < _padded_height; ++j)
            {
                for (int i = 0; i < _padded_width; ++i)
                {
                    *q++ = 0;
                    *q++ = 0;
                    *q++ = 0;
                    *q++ = 0;
                }
            }
        }
        break;

        case RS2_FORMAT_Y8:
        {
            uint8_t* from = (uint8_t*)frame_data_in;
            uint8_t* p = nullptr;
            uint8_t* q = (uint8_t*)frame_data_out;

            for (int j = 0; j < _real_height; ++j)
            {
                for (int i = 0; i < _real_width; ++i)
                {
                    p = from + scale * (j * width_in + i);
                    sum = 0;
                    for (size_t n = 0; n < scale; ++n)
                    {
                        for (size_t m = 0; m < scale; ++m)
                            sum += p[m];

                        p += width_in;
                    }

                    *q++ = (uint8_t)(sum / patch_size);
                }

                for (int i = _real_width; i < _padded_width; ++i)
                    *q++ = 0;
            }

            for (int j = _real_height; j < _padded_height; ++j)
            {
                for (int i = 0; i < _padded_width; ++i)
                    *q++ = 0;
            }
        }
        break;

        case RS2_FORMAT_Y16:
        {
            uint16_t* from = (uint16_t*)frame_data_in;
            uint16_t* p = nullptr;
            uint16_t* q = (uint16_t*)frame_data_out;

            for (int j = 0; j < _real_height; ++j)
            {
                for (int i = 0; i < _real_width; ++i)
                {
                    p = from + scale * (j * width_in + i);
                    sum = 0;
                    for (size_t n = 0; n < scale; ++n)
                    {
                        for (size_t m = 0; m < scale; ++m)
                            sum += p[m];

                        p += width_in;
                    }

                    *q++ = (uint16_t)(sum / patch_size);
                }

                for (int i = _real_width; i < _padded_width; ++i)
                    *q++ = 0;
            }

            for (int j = _real_height; j < _padded_height; ++j)
            {
                for (int i = 0; i < _padded_width; ++i)
                    *q++ = 0;
            }
        }
        break;

        default:
            break;
        }
    }
}

