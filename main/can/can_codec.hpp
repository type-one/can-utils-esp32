/*==========================================================================================
    Copyright (c) 2001-2023 Mireo, EU

    Extremely efficient DBC file format parser, built on top of Boost Spirit.

    Strictly follows the grammar rules from the last updated DBC specification available at
    http://mcu.so/Microcontroller/Automotive/dbc-file-format-documentation_compress.pdf
===========================================================================================*/

/**
 * @file can_codec.hpp
 * @brief Header file for CAN signal encoding and decoding, and physical value conversion.
 *
 * This file contains the definitions of classes and functions for encoding and decoding
 * CAN bus signals, handling calculations on CAN signal values, and converting raw signal
 * values to physical values. It is designed for use with the ESP32 micro-controller and
 * is a fork and modification of the MIREO version.
 *
 * @copyright Copyright (c) 2001-2023 Mireo, EU
 *
 * @details
 *
 * The main classes provided in this file are:
 * - can::sig_codec: A class to encode and decode CAN bus signals.
 * - can::sig_calc_type: A template class to handle calculations on CAN signal values.
 * - can::phys_value: A class to convert raw signal values to physical values.
 *
 * The file also defines an enumeration for value types (can::val_type_t).
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

#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <type_traits>
#include <variant>

#include "boost/endian.hpp"
#include "can_kernel.hpp"
namespace can
{
    /**
     * @class sig_codec
     * @brief A class to encode and decode CAN bus signals.
     */
    class sig_codec
    {
    private:
        using order = boost::endian::order;
        unsigned _start_bit, _bit_size;
        order _byte_order;
        char _sign_type;
        unsigned _byte_pos, _bit_pos, _last_bit_pos, _nbytes;

    public:
        /**
         * @brief Constructor for sig_codec.
         * @param sb Start bit.
         * @param bs Bit size.
         * @param bo Byte order ('0' for big endian, '1' for little endian).
         * @param st Sign type ('-' for signed, '+' for unsigned).
         */
        sig_codec(unsigned sb, unsigned bs, char bo, char st);

        /**
         * @brief Decode the CAN signal from raw data.
         * @param data Pointer to the raw data.
         * @return Decoded signal value.
         */
        std::uint64_t operator()(const std::uint8_t* data) const;

        /**
         * @brief Encode the CAN signal to raw data.
         * @param raw The raw signal value.
         * @param buffer Pointer to the buffer where the encoded data will be stored.
         */
        void operator()(std::uint64_t raw, void* buffer) const;

        /**
         * @brief Get the sign type of the signal.
         * @return Sign type ('-' for signed, '+' for unsigned).
         */
        char sign_type() const;
    };

    /**
     * @class sig_calc_type
     * @brief A class to handle calculations on CAN signal values.
     * @tparam T The type of the signal value (std::uint64_t, std::int64_t, float, or double).
     */
    template <typename T>
    requires std::is_same_v<T, std::uint64_t> || std::is_same_v<T, std::int64_t> || std::is_same_v<T,
        float> || std::is_same_v<T, double>
    class sig_calc_type
    {
        T _value = 0;

    public:
        /**
         * @brief Constructor for sig_calc_type.
         * @param raw The raw signal value.
         */
        sig_calc_type(std::uint64_t raw)
        {
            _value = *(const T*)&raw;
        }

        /**
         * @brief Get the raw signal value.
         * @return Raw signal value.
         */
        std::uint64_t get_raw() const
        {
            return *(const std::uint64_t*)&_value;
        }

        /**
         * @brief Add two sig_calc_type values.
         * @param rhs The right-hand side value to add.
         * @return The result of the addition.
         */
        sig_calc_type<T> operator+(const sig_calc_type<T>& rhs) const
        {
            return from_value(_value + rhs._value);
        }

        /**
         * @brief Compare (less) two sig_calc_type values.
         * @param rhs The right-hand side value to compare.
         * @return The result of comparison.
         */
        bool operator<(const sig_calc_type<T>& rhs) const
        {
            return _value < rhs._value;
        }

        /**
         * @brief Compare (less or equal) two sig_calc_type values.
         * @param rhs The right-hand side value to compare.
         * @return The result of comparison.
         */
        bool operator<=(const sig_calc_type<T>& rhs) const
        {
            return _value <= rhs._value;
        }

        /**
         * @brief Compare (greater) two sig_calc_type values.
         * @param rhs The right-hand side value to compare.
         * @return The result of comparison.
         */
        bool operator>(const sig_calc_type<T>& rhs) const
        {
            return _value > rhs._value;
        }

        /**
         * @brief Compare (greater or equal) two sig_calc_type values.
         * @param rhs The right-hand side value to compare.
         * @return The result of comparison.
         */
        bool operator>=(const sig_calc_type<T>& rhs) const
        {
            return _value >= rhs._value;
        }

        /**
         * @brief Compare (equal) two sig_calc_type values.
         * @param rhs The right-hand side value to compare.
         * @return The result of comparison.
         */
        bool operator==(const sig_calc_type<T>& rhs) const
        {
            return _value == rhs._value;
        }

        /**
         * @brief Compare (diff) two sig_calc_type values.
         * @param rhs The right-hand side value to compare.
         * @return The result of comparison.
         */
        bool operator!=(const sig_calc_type<T>& rhs) const
        {
            return _value != rhs._value;
        }

        /**
         * @brief Integer division with rounding.
         * @param d The divisor.
         * @return The result of the division.
         */
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
        /**
         * @brief Create a sig_calc_type from a value.
         * @param val The value.
         * @return The created sig_calc_type.
         */
        sig_calc_type<T> from_value(T val) const
        {
            return sig_calc_type(*(const std::uint64_t*)&val);
        }
    };

    /**
     * @enum val_type_t
     * @brief Enumeration for value types.
     */
    enum val_type_t
    {
        i64 = 0, ///< 64-bit signed integer
        f32 = 1, ///< 32-bit floating point
        f64 = 2, ///< 64-bit floating point
        u64 = 3  ///< 64-bit unsigned integer
    };

    /**
     * @class phys_value
     * @brief A class to convert raw signal values to physical values.
     */
    class phys_value
    {
        double _factor, _offset;

    public:
        /**
         * @brief Constructor for phys_value.
         * @param factor The factor for conversion.
         * @param offset The offset for conversion.
         */
        phys_value(double factor, double offset);

        /**
         * @brief Convert raw signal value to physical value.
         * @param raw The raw signal value.
         * @param val_type The type of the value.
         * @return The physical value.
         */
        double operator()(std::uint64_t raw, val_type_t val_type) const;

    private:
        /**
         * @brief Convert raw signal value to physical value.
         * @tparam T The type of the value.
         * @param raw The raw signal value.
         * @return The physical value.
         */
        template <typename T>
        double raw_to_phys(std::uint64_t raw) const
        {
            auto draw = double(*reinterpret_cast<T*>(&raw));
            return draw * _factor + _offset;
        }
    };

} // end namespace can
