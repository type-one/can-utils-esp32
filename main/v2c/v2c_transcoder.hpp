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

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <ranges>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>


#include "any/any.hpp"
#include "can/can_codec.hpp"
#include "can/frame_packet.hpp"
#include "dbc/dbc_parser.hpp"
#include "dbc/parser_template.hpp"

namespace can
{

    /**
     * @brief A constexpr structure that provides an assembly callable protocol object (CPO).
     *
     * This structure defines a type-erased signature and an overloaded function call operator
     * that uses tag dispatch to invoke the appropriate function based on the type of the first argument.
     */
    constexpr struct assemble_cpo
    {
        /**
         * @brief Type-erased signature for the callable protocol object.
         *
         * This defines the signature of the function that will be called using tag dispatch.
         */
        using type_erased_signature_t = std::uint64_t(mireo::this_&, std::int64_t, std::uint64_t);

        /**
         * @brief Overloaded function call operator.
         *
         * This operator uses tag dispatch to invoke the appropriate function based on the type of the first argument.
         *
         * @tparam T The type of the first argument.
         * @param x The first argument, which will be used to determine the appropriate function to call.
         * @param mux_val A 64-bit signed integer parameter.
         * @param fd A 64-bit unsigned integer parameter.
         * @return std::uint64_t The result of the invoked function.
         */
        template <typename T>
        requires mireo::tag_invocable<assemble_cpo, T&, std::uint64_t, std::uint64_t> std::uint64_t operator()(
            T& x, std::int64_t mux_val, std::uint64_t fd) const
        {
            return mireo::tag_invoke(*this, x, mux_val, fd);
        }
    } sig_assemble;

    /**
     * @brief A constexpr structure representing a reset customization point object (CPO).
     *
     * This structure defines a type-erased signature and an overloaded function call operator
     * that invokes the `reset` operation on the given object using tag dispatching.
     *
     * @tparam T The type of the object on which the reset operation is to be performed.
     *
     * @note The `reset` operation is performed using the `mireo::tag_invoke` mechanism.
     */
    constexpr struct reset_cpo
    {
        using type_erased_signature_t = void(mireo::this_&);

        template <typename T>
        requires mireo::tag_invocable<reset_cpo, T&>
        void operator()(T& x) const
        {
            return mireo::tag_invoke(*this, x);
        }
    } sig_reset;

    /**
     * @brief A type alias for the `assemble_cpo` and `reset_cpo` customization point objects.
     *
     * This type alias defines a type that can hold either the `assemble_cpo` or the `reset_cpo` object.
     */
    using sig_asm = mireo::any<sig_assemble, sig_reset>;

    /**
     * @class tr_signal
     * @brief A class to handle the transcoding of signals received from a CAN bus.
     *
     * This class is responsible for managing the name, codec, aggregation type,
     * value type, and optional multiplexer value of a signal. It provides methods
     * to decode and encode signal data, check if the signal is active based on
     * a frame multiplexer value, and get/set various properties of the signal.
     */
    class tr_signal
    {
    private:
        std::string _name; ///< Name of the signal.
        std::string _unit; ///< Unit of the signal.
        sig_codec _codec;  ///< Codec used for encoding/decoding the signal.

        std::string _agg_type = "LAST";       ///< Aggregation type of the signal.
        val_type_t _val_type = i64;           ///< Value type of the signal.
        std::optional<std::int64_t> _mux_val; ///< Optional multiplexer value.

    public:
        /**
         * @brief Constructs a tr_signal object.
         * @param name Name of the signal.
         * @param unit Unit of the signal.
         * @param codec Codec used for encoding/decoding the signal.
         * @param mux_val Optional multiplexer value.
         */
        tr_signal(std::string name, std::string unit, sig_codec codec, std::optional<std::int64_t> mux_val)
            : _name(std::move(name))
            , _unit(std::move(unit))
            , _codec(codec)
            , _mux_val(mux_val)
        {
        }

        /**
         * @brief Gets the name of the signal.
         * @return The name of the signal.
         */
        const std::string& name() const
        {
            return _name;
        }

        /**
         * @brief Gets the unit of the signal.
         * @return The unit of the signal.
         */
        const std::string& unit() const
        {
            return _unit;
        }

        /**
         * @brief Gets the optional multiplexer value.
         * @return The optional multiplexer value.
         */
        std::optional<int64_t> mux_val() const
        {
            return _mux_val;
        }

        /**
         * @brief Checks if the signal is active based on the frame multiplexer value.
         * @param frame_mux_val The frame multiplexer value.
         * @return True if the signal is active, false otherwise.
         */
        bool is_active(std::uint64_t frame_mux_val) const
        {
            return !_mux_val || _mux_val == frame_mux_val;
        }

        /**
         * @brief Gets the aggregation type of the signal.
         * @return The aggregation type of the signal.
         */
        std::string_view agg_type() const
        {
            return _agg_type;
        }

        /**
         * @brief Sets the aggregation type of the signal.
         * @param agg_type The new aggregation type.
         */
        void agg_type(const std::string& agg_type)
        {
            _agg_type = agg_type;
        }

        /**
         * @brief Gets the value type of the signal.
         * @return The value type of the signal.
         */
        val_type_t value_type() const
        {
            return _val_type;
        }

        /**
         * @brief Sets the value type of the signal.
         * @param vt The new value type.
         */
        void value_type(unsigned vt)
        {
            _val_type = val_type_t(vt);
            if (_val_type == i64 && _codec.sign_type() == '+')
                _val_type = u64;
        }


        /**
         * @brief Decodes the given data using the codec.
         * @param data The data to decode.
         * @return The decoded value.
         */
        std::uint64_t decode(std::uint64_t data) const
        {
            return _codec((std::uint8_t*)&data);
        }

        /**
         * @brief Encodes the given raw value using the codec.
         * @param raw The raw value to encode.
         * @return The encoded value.
         */
        std::uint64_t encode(std::uint64_t raw) const
        {
            std::uint64_t rv = 0;
            _codec(raw, (void*)&rv);
            return rv;
        }
    };

    /**
     * @class tr_muxer
     * @brief A class for encoding and decoding signals from a CAN bus.
     *
     * This class aggregates signals coming from a CAN bus using a specified codec.
     * It provides methods to encode and decode data.
     */

    class tr_muxer
    {
        sig_codec _codec;

    public:
        /**
         * @brief Constructs a tr_muxer object with the given codec.
         *
         * @param codec The codec to be used for encoding and decoding signals.
         */
        tr_muxer(sig_codec codec)
            : _codec(codec)
        {
        }

        /**
         * @brief Decodes the given data using the codec.
         *
         * @param data The data to be decoded.
         * @return The decoded data.
         */
        std::uint64_t decode(std::uint64_t data) const
        {
            return _codec((std::uint8_t*)&data);
        }

        /**
         * @brief Encodes the given raw data using the codec.
         *
         * @param raw The raw data to be encoded.
         * @return The encoded data.
         */
        std::uint64_t encode(std::uint64_t raw) const
        {
            std::uint64_t rv = 0;
            _codec(raw, (void*)&rv);
            return rv;
        }
    };

    class tr_message;

    /**
     * @class tx_group
     * @brief Manages a group of CAN messages for transmission.
     *
     * This class is responsible for collecting, managing, and publishing
     * CAN messages within a specified time interval. It provides methods
     * to add messages, check if all messages have been collected, and
     * publish the collected messages.
     */
    class tx_group
    {
        /**
         * @struct stamped_msg
         * @brief Represents a CAN message with a timestamp.
         *
         * This structure holds the timestamp, message ID, multiplexer value,
         * and message data for a CAN message.
         */
        struct stamped_msg
        {
            can_time stamp;
            canid_t message_id;
            std::int64_t message_mux;
            std::uint64_t mdata;
        };

        std::string _name;
        std::chrono::milliseconds _assemble_freq;
        can_time _group_origin;

        std::vector<stamped_msg> _msg_clumps;

        friend class tr_message;

    public:
        /**
         * @brief Constructs a tx_group object.
         * @param name The name of the group.
         * @param assemble_freq The frequency at which messages are assembled.
         */
        tx_group(std::string_view name, std::uint32_t assemble_freq)
            : _name(name)
            , _assemble_freq(assemble_freq)
        {
        }

        /**
         * @brief Gets the name of the group.
         * @return The name of the group.
         */
        std::string_view name() const
        {
            return _name;
        }

        /**
         * @brief Sets the origin time for the group.
         * @param tp The origin time point.
         */
        void time_begin(can_time tp)
        {
            _group_origin = tp;
        }

        /**
         * @brief Attempts to publish the collected CAN frames up to a specified time point.
         * @param up_to The time point up to which frames should be published.
         * @param fp The frame packet to which frames are appended.
         */
        void try_publish(can_time up_to, frame_packet& fp);

    private:
        /**
         * @brief Publishes the collected CAN frames.
         * @param tp The time point at which frames are published.
         * @param fp The frame packet to which frames are appended.
         */
        void publish(can_time tp, frame_packet& fp);

        /**
         * @brief Checks if all messages in the group have been collected within the interval.
         * @return True if all messages have been collected, false otherwise.
         */
        bool all_collected() const;

        /**
         * @brief Assigns a message ID and multiplexer value to the group.
         * @param message_id The message ID.
         * @param message_mux The multiplexer value.
         */
        void assign(canid_t message_id, std::int64_t message_mux);

        /**
         * @brief Checks if a timestamp is within the group's interval.
         * @param stamp The timestamp to check.
         * @return True if the timestamp is within the interval, false otherwise.
         */
        bool within_interval(can_time stamp) const;

        /**
         * @brief Adds a clumped message to the group.
         * @param stamp The timestamp of the message.
         * @param message_id The message ID.
         * @param message_mux The multiplexer value.
         * @param cval The clumped value.
         */
        void add_clumped(can_time stamp, canid_t message_id, std::int64_t message_mux, std::uint64_t cval);
    };

    /**
     * @class tr_message
     * @brief Represents a CAN message with associated signals and optional multiplexer.
     *
     * The tr_message class provides functionality to manage and process CAN messages,
     * including assigning groups, assembling messages from CAN frames, and managing signals.
     * It supports adding signals and multiplexers, setting signal aggregation and value types,
     * and retrieving active signals based on a frame descriptor.
     */
    class tr_message
    {
        std::vector<tr_signal> _signals; /// A vector of signals associated with the message.
        std::optional<tr_muxer> _mux;    /// An optional multiplexer associated with the message.

        std::vector<sig_asm> _sig_asms; /// A vector of signal assemblers for the message.
        tx_group* _tx_group = nullptr;  /// A pointer to the transmission group assigned to the message.
        can_time _last_stamp;           /// The timestamp of the last CAN frame processed.

    public:
        /**
         * @brief Assigns a group to the message.
         * @param txg The group to be assigned.
         * @param message_id The message ID.
         */
        void assign_group(tx_group* txg, std::uint32_t message_id);

        /**
         * @brief Assembles the message from the CAN frame.
         * @param stamp The timestamp of the frame.
         * @param frame The CAN frame.
         */
        void assemble(can_time stamp, can_frame frame);

        /**
         * @brief Sets the aggregation type for a signal.
         * @param sig_name The name of the signal.
         * @param agg_type The aggregation type.
         */
        void sig_agg_type(const std::string& sig_name, const std::string& agg_type);

        /**
         * @brief Sets the value type for a signal.
         * @param sig_name The name of the signal.
         * @param sig_ext_val_type The external value type.
         */
        void sig_val_type(const std::string& sig_name, unsigned sig_ext_val_type);

        /**
         * @brief Adds a signal to the message.
         * @param sig The signal to be added.
         */
        void add_signal(tr_signal sig);

        /**
         * @brief Adds a multiplexer to the message.
         * @param mux The multiplexer to be added.
         */
        void add_muxer(tr_muxer mux);

        /**
         * @brief Filters and returns active signals based on the provided frame data.
         *
         * This function takes a 64-bit frame data (fd) as input and decodes it using
         * the optional multiplexer (_mux). If the multiplexer is present, it decodes
         * the frame data to obtain a frame multiplexer value (frame_mux). If the
         * multiplexer is not present, frame_mux is set to -1.
         *
         * The function then filters the _signals collection to include only those
         * signals that are active based on the frame_mux value.
         *
         * @param fd The 64-bit frame data to be decoded and used for filtering signals.
         * @return A filtered range of signals that are active based on the frame_mux value.
         */
        auto signals(std::uint64_t fd) const
        {
            std::uint64_t frame_mux = _mux.has_value() ? _mux->decode(fd) : -1;
            // Pipe operator: A beginner's guide to C++ Ranges and Views.
            // https://hannes.hauswedell.net/post/2019/11/30/range_intro/
            return _signals
                | std::ranges::views::filter([frame_mux](const auto& sig) { return sig.is_active(frame_mux); });
        }

    private:
        /**
         * @brief Creates signal assemblers for the message.
         */
        void make_sig_assemblers();

        /**
         * @brief Resets the signal assemblers.
         */
        void reset_sig_asms();

        /**
         * @brief Finds a signal by its name.
         * @param sig_name The name of the signal.
         * @return Pointer to the signal if found, nullptr otherwise.
         */
        tr_signal* find_signal(const std::string& sig_name);

        /**
         * @brief Gets the distinct multiplexer values from the message's signals.
         * @return A vector of distinct multiplexer values.
         */
        std::vector<std::uint64_t> distinct_mux_vals() const;
    };

    /**
     * @class vin_assembler
     * @brief A class to assemble Vehicle Identification Numbers (VIN) from CAN frames.
     *
     * This class provides functionality to decode VIN characters from CAN frames and
     * assemble them into a complete VIN string.
     */
    class vin_assembler
    {
        static constexpr std::size_t vin_len = 17; // industry standard
        std::uint32_t _vin_msg_id;
        std::uint32_t _cbits = 0;
        char _vin[vin_len];

    public:
        /**
         * @brief Checks if the VIN assembler is empty.
         * @return True if the VIN assembler is empty, false otherwise.
         */
        bool empty() const
        {
            return _cbits != ((1 << vin_len) - 1);
        }

        /**
         * @brief Retrieves the assembled VIN value.
         * @return The assembled VIN string if complete, otherwise an empty string.
         */
        std::string value() const
        {
            return empty() ? std::string {} : std::string { _vin, _vin + vin_len };
        }

        /**
         * @brief Sets the message ID for VIN messages.
         * @param id The message ID to set.
         */
        void vin_message_id(std::uint32_t id)
        {
            _vin_msg_id = id;
        }

        /**
         * @brief Decodes some VIN characters from the CAN frame.
         * @param msg The message containing the signals.
         * @param frame The CAN frame.
         * @return True if new characters were decoded, false otherwise.
         */
        bool decode_some(const can::tr_message& msg, can_frame frame);

    private:
        /**
         * @brief Gets the character index for a VIN signal.
         * @param sig_name The name of the signal.
         * @return The character index, or 0 if not a VIN signal.
         */
        static int vin_char(std::string_view sig_name);
    };

    /**
     * @class v2c_transcoder
     * @brief A class responsible for transcoding CAN frames to frame packets and managing transmission groups, signals,
     * and messages.
     *
     * This class provides functionalities to transcode CAN frames, manage transmission groups, signals, and messages,
     * set environment variables, and configure signal value and aggregation types. It also handles timers and stores
     * assembled CAN frames up to a specified time point.
     */
    class v2c_transcoder
    {
        std::chrono::milliseconds _publish_freq; /// The frequency at which data is published.
        /// The frequency at which data is updated, calculated as the greatest common divisor (GCD) of all transmission
        /// groups' frequencies.
        std::chrono::milliseconds _update_freq { 0 }; // gcd of all tx_groups' freqs

        std::unordered_map<canid_t, tr_message> _msgs;     /// A map of CAN message IDs to their corresponding messages.
        std::vector<std::unique_ptr<tx_group>> _tx_groups; /// vector of unique pointers to transmission groups.
        vin_assembler _vin;                                /// An assembler for the vehicle identification number (VIN).

        frame_packet _frame_packet; /// A packet containing the transcoded frame data.
        can_time _last_update_tp;   /// The time point of the last update.

    public:
        /**
         * @brief Transcodes a CAN frame to a frame packet.
         * @param stamp The timestamp of the frame.
         * @param frame The CAN frame.
         * @return The frame packet.
         */
        frame_packet transcode(can_time stamp, can_frame frame);

        /**
         * @brief Retrieves the vehicle identification number (VIN).
         * @return The VIN as a string.
         */
        std::string vin() const
        {
            return _vin.value();
        }

        /**
         * @brief Assigns a transmission group to a message.
         * @param object_type The type of the object.
         * @param message_id The message ID.
         * @param tx_group The transmission group.
         */
        void assign_tx_group(const std::string& object_type, unsigned message_id, const std::string& tx_group);

        /**
         * @brief Adds a signal to a message.
         * @param message_id The message ID.
         * @param sig The signal to be added.
         */
        void add_signal(canid_t message_id, tr_signal sig);

        /**
         * @brief Adds a multiplexer to a message.
         * @param message_id The message ID.
         * @param mux The multiplexer to be added.
         */
        void add_muxer(canid_t message_id, tr_muxer mux);

        /**
         * @brief Adds a message to the transcoder.
         * @param message_id The message ID.
         * @param message_name The name of the message.
         */
        void add_message(canid_t message_id, std::string_view message_name);

        /**
         * @brief Sets an environment variable for the transcoder.
         * @param name The name of the environment variable.
         * @param ev_value The value of the environment variable.
         */
        void set_env_var(const std::string& name, std::int64_t ev_value);

        /**
         * @brief Sets the value type for a signal in a message.
         * @param message_id The message ID.
         * @param sig_name The name of the signal.
         * @param sig_ext_val_type The external value type.
         */
        void set_sig_val_type(canid_t message_id, const std::string& sig_name, unsigned sig_ext_val_type);

        /**
         * @brief Sets the aggregation type for a signal in a message.
         * @param message_id The message ID.
         * @param sig_name The name of the signal.
         * @param agg_type The aggregation type.
         */
        void set_sig_agg_type(canid_t message_id, const std::string& sig_name, const std::string& agg_type);

        /**
         * @brief Sets up timers for the transcoder.
         * @param first_stamp The first timestamp.
         */
        void setup_timers(can_time first_stamp);

        /**
         * @brief Stores the assembled CAN frames up to a specified time point.
         * @param up_to The time point up to which frames should be stored.
         */
        void store_assembled(can_time up_to);

        /**
         * @brief Finds a message by its ID.
         * @param message_id The message ID.
         * @return Pointer to the message if found, nullptr otherwise.
         */
        tr_message* find_message(canid_t message_id);
    };

    // tag-invokes used by dbc_parser.cpp

    /**
     * @brief Invokes the tag_invoke function to add a signal to the v2c_transcoder.
     *
     * @param def_sg_cpo Customization point object for tag_invoke.
     * @param this_ Reference to the v2c_transcoder instance.
     * @param message_id The ID of the message to which the signal belongs.
     * @param sg_mux_switch_val Optional multiplexer switch value.
     * @param sg_name The name of the signal.
     * @param sg_start_bit The starting bit position of the signal.
     * @param sg_size The size of the signal in bits.
     * @param sg_byte_order The byte order of the signal ('0' for little-endian, '1' for big-endian).
     * @param sg_sign The sign of the signal ('0' for unsigned, '1' for signed).
     * @param sg_factor The factor to be applied to the signal value.
     * @param sg_offset The offset to be applied to the signal value.
     * @param sg_min The minimum value of the signal.
     * @param sg_unit The unit of the signal.
     * @param rec_ords A vector of record orders.
     */
    inline void tag_invoke(def_sg_cpo, v2c_transcoder& this_, std::uint32_t message_id,
        std::optional<unsigned> sg_mux_switch_val, std::string sg_name, unsigned sg_start_bit, unsigned sg_size,
        char sg_byte_order, char sg_sign, double sg_factor, double sg_offset, double sg_min, double sg_max,
        std::string sg_unit, std::vector<size_t> rec_ords)
    {
        (void)sg_factor;
        (void)sg_offset;
        (void)sg_min;
		(void)sg_max;
        (void)rec_ords;

        sig_codec codec { sg_start_bit, sg_size, sg_byte_order, sg_sign };
        tr_signal sig { sg_name, sg_unit, codec, std::optional<std::int64_t>(sg_mux_switch_val) };
        this_.add_signal(message_id, std::move(sig));
    }

    /**
     * @brief Tag invoke function for v2c_transcoder.
     *
     * This function is used to handle the tag_invoke operation for the v2c_transcoder class.
     *
     * @param def_sg_mux_cpo A tag type used to identify the operation.
     * @param this_ Reference to the v2c_transcoder instance.
     * @param message_id The message ID associated with the signal.
     * @param sg_name The name of the signal (unused).
     * @param sg_start_bit The starting bit position of the signal.
     * @param sg_size The size of the signal in bits.
     * @param sg_byte_order The byte order of the signal ('L' for little-endian, 'B' for big-endian).
     * @param sg_sign The sign of the signal ('S' for signed, 'U' for unsigned).
     * @param sg_unit The unit of the signal (unused).
     * @param rec_ords A vector of record orders (unused).
     */
    inline void tag_invoke(def_sg_mux_cpo, v2c_transcoder& this_, std::uint32_t message_id, std::string sg_name,
        unsigned sg_start_bit, unsigned sg_size, char sg_byte_order, char sg_sign, std::string sg_unit,
        std::vector<size_t> rec_ords)
    {
        (void)sg_name;
        (void)sg_unit;
        (void)rec_ords;
        sig_codec codec { sg_start_bit, sg_size, sg_byte_order, sg_sign };
        tr_muxer mux { codec };
        this_.add_muxer(message_id, std::move(mux));
    }

    /**
     * @brief Custom tag_invoke function for v2c_transcoder.
     *
     * This function is used to add a message to the v2c_transcoder instance.
     *
     * @param def_bo_cpo A customization point object (CPO) for tag_invoke.
     * @param this_ Reference to the v2c_transcoder instance.
     * @param message_id The ID of the message to be added.
     * @param msg_name The name of the message to be added.
     * @param msg_size The size of the message (unused).
     * @param transmitter_ord The order of the transmitter (unused).
     */
    inline void tag_invoke(def_bo_cpo, v2c_transcoder& this_, std::uint32_t message_id, std::string msg_name,
        std::size_t msg_size, std::size_t transmitter_ord)
    {
        (void)msg_size;
        (void)transmitter_ord;
        this_.add_message(message_id, std::move(msg_name));
    }

    /**
     * @brief Invokes the tag_invoke function to set an environment variable.
     *
     * This function is an inline implementation of the tag_invoke function, which sets an environment variable
     * using the provided parameters. The function ignores most of the parameters except for the name and initial value.
     *
     * @param def_ev_cpo A placeholder parameter for the customization point object.
     * @param this_ A reference to the v2c_transcoder object.
     * @param name The name of the environment variable to set.
     * @param type The type of the environment variable (ignored).
     * @param ev_min The minimum value of the environment variable (ignored).
     * @param ev_max The maximum value of the environment variable (ignored).
     * @param unit The unit of the environment variable (ignored).
     * @param initial The initial value of the environment variable.
     * @param ev_id The ID of the environment variable (ignored).
     * @param access_type The access type of the environment variable (ignored).
     * @param access_nodes_ords The access nodes ordinals of the environment variable (ignored).
     */
    inline void tag_invoke(def_ev_cpo, v2c_transcoder& this_, std::string name, unsigned type, double ev_min,
        double ev_max, std::string unit, double initial, unsigned ev_id, std::string access_type,
        std::vector<std::size_t> access_nodes_ords)
    {
        (void)type;
        (void)ev_min;
        (void)ev_max;
        (void)unit;
        (void)ev_id;
        (void)access_type;
        (void)access_nodes_ords;

        this_.set_env_var(name, initial);
    }

    /**
     * @brief Handles the invocation of a tag with specific attributes and values.
     *
     * This function is called when a tag is invoked with the specified attributes and values.
     * It processes the tag based on the attribute name and value, and performs specific actions
     * such as setting the signal aggregation type or assigning a transmission group.
     *
     * @param def_ba_cpo A placeholder parameter for the tag invocation.
     * @param this_ Reference to the v2c_transcoder object.
     * @param attr_name The name of the attribute being processed.
     * @param object_type The type of the object associated with the tag.
     * @param object_name The name of the object associated with the tag.
     * @param bu_id A placeholder parameter for the buffer ID (unused).
     * @param message_id The ID of the message associated with the tag.
     * @param attr_val The value of the attribute, which can be an integer, double, or string.
     */
    inline void tag_invoke(def_ba_cpo, v2c_transcoder& this_, std::string attr_name, std::string object_type,
        std::string object_name, std::size_t bu_id, unsigned message_id,
        std::variant<std::int32_t, double, std::string> attr_val)
    {
        (void)bu_id;

        if (attr_name == "AggType" && attr_val.index() == 2)
        {
            this_.set_sig_agg_type(message_id, object_name, std::get<std::string>(attr_val));
        }

        if (attr_name == "TxGroupFreq" && object_type == "BO_")
        {
            this_.assign_tx_group(object_type, message_id, std::get<std::string>(attr_val));
        }
    }

    /**
     * @brief Invokes the tag_invoke function to set the signal value type.
     *
     * This function is an inline implementation of the tag_invoke function, which
     * sets the signal value type for a given message ID and signal name.
     *
     * @param def_sig_valtype_cpo A customization point object (CPO) for defining the signal value type.
     * @param this_ A reference to the v2c_transcoder object.
     * @param message_id The ID of the message.
     * @param sig_name The name of the signal.
     * @param sig_ext_val_type The external value type of the signal.
     */
    inline void tag_invoke(def_sig_valtype_cpo, v2c_transcoder& this_, unsigned message_id, std::string sig_name,
        unsigned sig_ext_val_type)
    {
        this_.set_sig_val_type(message_id, sig_name, sig_ext_val_type);
    }

} // end namesapce can
