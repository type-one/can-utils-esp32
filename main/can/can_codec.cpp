/*==========================================================================================
    Copyright (c) 2001-2023 Mireo, EU

    Extremely efficient DBC file format parser, built on top of Boost Spirit.

    Strictly follows the grammar rules from the last updated DBC specification available at
    http://mcu.so/Microcontroller/Automotive/dbc-file-format-documentation_compress.pdf
===========================================================================================*/

/**
 * @file can_codec.cpp
 * @brief Implementation file for CAN signal encoding and decoding, and physical value conversion.
 * 
 * This file contains the implementations of classes and functions for encoding and decoding
 * CAN bus signals, handling calculations on CAN signal values, and converting raw signal
 * values to physical values. It is designed for use with the ESP32 micro-controller and
 * is a fork and modification of the MIREO version.
 *
 * @copyright Copyright (c) 2001-2023 Mireo, EU
 * 
 */

//-----------------------------------------------------------------------------//
// ESP32 C++ DBC/CAN parser - Spare time mod and FreeRTOS port for fun         //
// Laurent Lardinois https://be.linkedin.com/in/laurentlardinois               //
//                                                                             //
// https://github.com/type-one/can-utils-esp32                                 //
//                                                                             //
// A C++ DBC file parser, and a CAN telemetry tool, adapted for                //
// ESP32 micro-controller, forked and modified from MIREO version at           //
// https://github.com/mireo/can-utils                                          //
//-----------------------------------------------------------------------------//

#include <cstdint>
#include <string>
#include <type_traits>
#include <variant>

#include "boost/endian.hpp"

#include "can_codec.hpp"
#include "can_kernel.hpp"

namespace can
{
    sig_codec::sig_codec(unsigned sb, unsigned bs, char bo, char st)
        : _start_bit(sb)
        , _bit_size(bs)
        , _byte_order(bo == '0' ? order::big : order::little)
        , _sign_type(st)
    {
        _byte_pos = _start_bit / 8;
        _bit_pos = _byte_order == order::little ? _start_bit % 8 : _start_bit - _byte_pos * 8;
        _last_bit_pos
            = _byte_order == order::little ? (_start_bit + _bit_size - 1) % 8 : (7 - _start_bit % 8) + _bit_size - 64;
        _nbytes = _byte_order == order::little ? (_bit_size + _bit_pos + 7) / 8
                                               : (_bit_size + (7 - _start_bit % 8) + 7) / 8;
    }

    std::uint64_t sig_codec::operator()(const std::uint8_t* data) const
    {
        std::uint64_t val = _byte_order == order::little ? boost::endian::load_little_u64(data + _byte_pos)
                                                         : boost::endian::load_big_u64(data + _byte_pos);

        if (_nbytes > 8)
        {
            std::uint64_t ninth_byte = data[_byte_pos + 8];
            if (_byte_order == order::little)
            {
                val >>= _bit_pos;
                ninth_byte &= (1ull << (_last_bit_pos + 1)) - 1;
                ninth_byte <<= 8 - _bit_pos + 7 * 8;
            }
            else
            {
                val &= (1ull << (_start_bit % 8 + 7 * 8) << 1ull) - 1;
                val <<= _last_bit_pos;
                ninth_byte >>= 8 - _last_bit_pos;
            }
            val |= ninth_byte;
        }
        else
        {
            std::uint64_t last_bit_pos = (8 * (7 - (_bit_pos / 8))) + (_start_bit % 8) - (_bit_size - 1);
            val = _byte_order == order::little ? val >> _bit_pos : val >> last_bit_pos;
            val &= (1ull << (_bit_size - 1) << 1ull) - 1;
        }

        if (_sign_type == '-')
        {
            std::uint64_t mask_signed = ~((1ull << (_bit_size - 1ull)) - 1);
            if (val & mask_signed)
            {
                val |= mask_signed;
            }
        }

        return val;
    }

    void sig_codec::operator()(std::uint64_t raw, void* buffer) const
    {
        char* b = reinterpret_cast<char*>(buffer);

        if (_byte_order == order::big)
        {
            std::uint64_t src = _start_bit;
            std::uint64_t dst = _bit_size - 1;
            for (std::uint64_t i = 0; i < _bit_size; i++)
            {
                if (raw & (1ull << dst))
                {
                    b[src / 8] |= 1ull << (src % 8);
                }
                else
                {
                    b[src / 8] &= ~(1ull << (src % 8));
                }
                if ((src % 8) == 0)
                {
                    src += 15;
                }
                else
                {
                    src--;
                }
                dst--;
            }
        }
        else
        {
            std::uint64_t src = _start_bit;
            std::uint64_t dst = 0;
            for (std::uint64_t i = 0; i < _bit_size; i++)
            {
                if (raw & (1ull << dst))
                {
                    b[src / 8] |= 1ull << (src % 8);
                }
                else
                {
                    b[src / 8] &= ~(1ull << (src % 8));
                }
                src++;
                dst++;
            }
        }
    }

    char sig_codec::sign_type() const
    {
        return _sign_type;
    }


    phys_value::phys_value(double factor, double offset)
        : _factor(factor)
        , _offset(offset)
    {
    }

    double phys_value::operator()(std::uint64_t raw, val_type_t val_type) const
    {
        switch (val_type)
        {
            case i64:
                return raw_to_phys<std::int64_t>(raw);
            case u64:
                return raw_to_phys<std::uint64_t>(raw);
            case f32:
                return raw_to_phys<float>(raw);
            case f64:
                return raw_to_phys<double>(raw);
        }
        return 0;
    }

} // end namespace can
