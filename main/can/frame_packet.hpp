/*==========================================================================================
    Copyright (c) 2001-2023 Mireo, EU

    Extremely efficient DBC file format parser, built on top of Boost Spirit.

    Strictly follows the grammar rules from the last updated DBC specification available at
    http://mcu.so/Microcontroller/Automotive/dbc-file-format-documentation_compress.pdf
===========================================================================================*/

//-----------------------------------------------------------------------------//
// ESP32 C++ DBC/CAN parser - Spare time development for fun                   //
// Laurent Lardinois https://be.linkedin.com/in/laurentlardinois               //
//                                                                             //
// https://github.com/type-one/can-utils-esp32                                 //
//                                                                             //
// A C++ DBC file parser, and a CAN telemetry tool, adapted for                //
// ESP32 micro-controller, forked and modified from MIREO version at           //
// https://github.com/mireo/can-utils                                          //
//-----------------------------------------------------------------------------//

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <vector>


#include "can/can_kernel.hpp"

/*

|DBC version (2 byte)|UTC (4 byte)|
|usec part (4 byte)|CAN frame|
|usec part (4 byte)|CAN frame|
...

*/

namespace can
{

    inline void use_non_muxed(can_frame& cf, bool use)
    {
        if (use)
        {
            cf.__res0 |= 0x1;
        }
        else
        {
            cf.__res0 &= ~0x1;
        }
    }

    inline bool use_non_muxed(can_frame& cf)
    {
        return cf.__res0 & 0x1;
    }

    using can_time = std::chrono::system_clock::time_point;

    class frame_packet
    {
        using base = std::vector<std::uint8_t>;
        base _buff;

    public:
        frame_packet()
        {
        }
        frame_packet(base buff)
            : _buff(std::move(buff))
        {
        }

        frame_packet(frame_packet&&) = default;
        frame_packet& operator=(frame_packet&&) = default;
        frame_packet(const frame_packet&) = delete;
        frame_packet& operator=(const frame_packet&) = delete;

        void prepare(std::uint32_t utc)
        {
            _buff.resize(0);
            _buff.reserve(32 * 1024);

            // DBC version
            constexpr std::uint16_t dbc_version = 100;
            append(dbc_version);

            append(utc);
        }

        std::uint32_t utc() const
        {
            return *(const std::uint32_t*)(_buff.data() + 2);
        }

        bool empty() const
        {
            return _buff.size() <= 6;
        }

        std::size_t byte_size() const
        {
            return _buff.size();
        }

        const std::uint8_t* data_begin() const
        {
            return _buff.data();
        }

        const std::uint8_t* data_end() const
        {
            return _buff.data() + _buff.size();
        }

        template <typename int_type>
        void append(int_type val)
        {
            const std::uint8_t* b = (const std::uint8_t*)&val;
            _buff.insert(_buff.end(), b, b + sizeof(int_type));
        }

        void append(can_frame frame)
        {
            const std::uint8_t* b = (const std::uint8_t*)&frame;
            _buff.insert(_buff.end(), b, b + sizeof(can_frame));
        }

        std::vector<uint8_t> release()
        {
            return std::move(_buff);
        }
    };


    class frame_iterator_sentinel
    {
    };

    class frame_iterator
    {
        const frame_packet& _frame_packet;
        std::uint32_t _packet_utc = 0;
        const std::uint8_t* _msg_iter;

    public:
        frame_iterator(const frame_packet& fp)
            : _frame_packet(fp)
        {
            _packet_utc = fp.utc();
            _msg_iter = fp.data_begin() + 6;
        }

        ~frame_iterator()
        {
        }

        bool operator==(const frame_iterator_sentinel&) const
        {
            if (_frame_packet.empty())
			{
                return true;
			}

            return _msg_iter == _frame_packet.data_end();
        }

        std::pair<can_time, can_frame> operator*()
        {
            using namespace std::chrono;
            std::int32_t millis = *(const std::int32_t*)_msg_iter;
            can_frame frame;

            std::memcpy(&frame, _msg_iter + 4, sizeof(can_frame));
            return { can_time(seconds(_packet_utc)) + milliseconds(millis), std::move(frame) };
        }

        frame_iterator& operator++()
        {
            if (_frame_packet.empty())
			{
                return *this;
			}

            _msg_iter += 4 + sizeof(can_frame);

            return *this;
        }
    };

    inline frame_iterator begin(const frame_packet& fp)
    {
        return frame_iterator(fp);
    }

    inline frame_iterator_sentinel end(const frame_packet&)
    {
        return {};
    }

} // end namespace can
