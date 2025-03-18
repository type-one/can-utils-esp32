/*==========================================================================================
    Copyright (c) 2001-2023 Mireo, EU

    Extremely efficient DBC file format parser, built on top of Boost Spirit.

    Strictly follows the grammar rules from the last updated DBC specification available at
    http://mcu.so/Microcontroller/Automotive/dbc-file-format-documentation_compress.pdf
===========================================================================================*/
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

#pragma once

#include <cstdint>
#include <string>
#include <type_traits>
#include <variant>


#include "boost/endian.hpp"

#include "can_kernel.hpp"

namespace can
{

    class sig_codec
    {
        using order = boost::endian::order;
        unsigned _start_bit, _bit_size;
        order _byte_order;
        char _sign_type;
        unsigned _byte_pos, _bit_pos, _last_bit_pos, _nbytes;

    public:
        sig_codec(unsigned sb, unsigned bs, char bo, char st)
            : _start_bit(sb)
            , _bit_size(bs)
            , _byte_order(bo == '0' ? order::big : order::little)
            , _sign_type(st)
        {
            _byte_pos = _start_bit / 8;
            _bit_pos = _byte_order == order::little ? _start_bit % 8 : _start_bit - _byte_pos * 8;
            _last_bit_pos = _byte_order == order::little ? (_start_bit + _bit_size - 1) % 8
                                                         : (7 - _start_bit % 8) + _bit_size - 64;
            _nbytes = _byte_order == order::little ? (_bit_size + _bit_pos + 7) / 8
                                                   : (_bit_size + (7 - _start_bit % 8) + 7) / 8;
        }

        std::uint64_t operator()(const std::uint8_t* data) const
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

        void operator()(std::uint64_t raw, void* buffer) const
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

        char sign_type() const
        {
            return _sign_type;
        }
    };

    template <typename T>
    requires std::is_same_v<T, std::uint64_t> || std::is_same_v<T, std::int64_t> || std::is_same_v<T, float> || std::is_same_v<T,
        double>
    class sig_calc_type
    {
        T _value = 0;

    public:
        sig_calc_type(std::uint64_t raw)
        {
            _value = *(const T*)&raw;
        }

        std::uint64_t get_raw() const
        {
            return *(const std::uint64_t*)&_value;
        }

        sig_calc_type<T> operator+(const sig_calc_type<T>& rhs) const
        {
            return from_value(_value + rhs._value);
        }

        sig_calc_type<T> idivround(std::int64_t d) const
        {
            if constexpr (std::is_same_v<T, std::uint64_t> || std::is_same_v<T, std::int64_t>)
			{
                return from_value(T((_value < 0) ? (_value - d / 2) / d : (_value + d / 2) / d));
			}
            else
			{
                return from_value(_value / d);
			}
        }

    private:
        sig_calc_type<T> from_value(T val) const
        {
            return sig_calc_type(*(const std::uint64_t*)&val);
        }
    };

    enum val_type_t
    {
        i64 = 0,
        f32 = 1,
        f64 = 2,
        u64 = 3
    };

    class phys_value
    {
        double _factor, _offset;

    public:
        phys_value(double factor, double offset)
            : _factor(factor)
            , _offset(offset)
        {
        }

        double operator()(std::uint64_t raw, val_type_t val_type) const
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

    private:
        template <typename T>
        double raw_to_phys(std::uint64_t raw) const
        {
            auto draw = double(*reinterpret_cast<T*>(&raw));
            return draw * _factor + _offset;
        }
    };

} // end namespace can
