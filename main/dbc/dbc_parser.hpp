/*==========================================================================================
    Copyright (c) 2001-2023 Mireo, EU

    Extremely efficient DBC file format parser, built on top of Boost Spirit.

    Strictly follows the grammar rules from the last updated DBC specification available at
    http://mcu.so/Microcontroller/Automotive/dbc-file-format-documentation_compress.pdf
===========================================================================================*/

/**
 * @file dbc_parser.hpp
 * @brief Header file - Extremely efficient DBC file format parser, built on top of Boost Spirit.
 *
 * This file contains the implementation of a DBC (Database Container) parser for CAN (Controller Area Network) data.
 * The parser is designed to be highly efficient and strictly follows the grammar rules from the latest DBC
 * specification. It is adapted for the ESP32 micro-controller and is a fork and modification of the MIREO version.
 *
 * @note The parser uses Boost Spirit for parsing the DBC file format.
 *
 * @copyright (c) 2001-2023 Mireo, EU
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

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>


#include "any/any.hpp"

namespace can
{

    /**
     * @brief A struct to represent a method name.
     *
     * This struct template is used to store a method name as a compile-time constant.
     *
     * @tparam N The size of the method name string.
     */
    template <std::size_t N>
    struct method_name
    {
        constexpr method_name(const char (&d)[N])
        {
            (void)d;
        }
    };

    /**
     * @brief A template struct representing a customization point object (CPO) for a given method.
     *
     * This struct provides a mechanism to invoke a method on an object using tag dispatching.
     *
     * @tparam method The method name to be invoked.
     * @tparam Args The argument types for the method.
     */
    template <method_name method, typename... Args>
    struct cpo
    {
        /**
         * @brief Type-erased signature for the method.
         */
        using type_erased_signature_t = void(mireo::this_&, Args...);

        /**
         * @brief Invokes the method on the given object if it is tag-invocable.
         *
         * @tparam T The type of the object.
         * @param x The object on which the method is invoked.
         * @param args The arguments to be passed to the method.
         */
        template <typename T>
        requires mireo::tag_invocable<cpo, T&, Args...>
        void operator()(T& x, Args... args) const
        {
            return mireo::tag_invoke(*this, x, std::forward<Args>(args)...);
        }

        /**
         * @brief Friend function to invoke the method on a reference-wrapped object.
         *
         * @tparam T The type of the object.
         * @param t The reference-wrapped object.
         * @param args The arguments to be passed to the method.
         */
        template <typename T>
        friend void tag_invoke(cpo<method, Args...>, std::reference_wrapper<T>& t, Args... args)
        {
            mireo::tag_invoke(cpo<method, Args...> {}, t.get(), std::forward<Args>(args)...);
        }


        /**
         * @brief Friend function to invoke the method on an object.
         *
         * This is a fallback function that does nothing if the method is not tag-invocable.
         *
         * @tparam T The type of the object.
         * @param args The arguments to be passed to the method.
         */
        template <typename T>
        friend void tag_invoke(cpo<method, Args...>, T&, Args...)
        {
        }
    };

    /**
     * @typedef def_version_cpo
     * @brief Custom property object for the version definition in a DBC file.
     *
     * This type alias defines a custom property object (CPO) named "version" used to represent the version
     * information in a DBC file.
     */
    using def_version_cpo = cpo<"version", std::string>;
    inline constexpr def_version_cpo def_version;

    /**
     * @typedef def_bu_cpo
     * @brief Custom property object for the bus unit definition in a DBC file.
     *
     * This type alias defines a custom property object (CPO) named "bu" used to represent the bus unit
     * information in a DBC file.
     */
    using def_bu_cpo = cpo<"bu", std::vector<std::string>>;
    inline constexpr def_bu_cpo def_bu;

    /**
     * @typedef def_bo_cpo
     * @brief Custom property object for the board definition in a DBC file.
     *
     * This type alias defines a custom property object (CPO) named "bo" used to represent the board
     * information in a DBC file.
     */
    using def_bo_cpo = cpo<"bo", std::uint32_t, std::string, std::size_t, std::size_t>;
    inline constexpr def_bo_cpo def_bo;

    /**
     * @typedef def_sg_cpo
     * @brief Custom property object for the signal definition in a DBC file.
     *
     * This type alias defines a custom property object (CPO) named "sg" used to represent the signal
     * information in a DBC file.
     */
    using def_sg_cpo = cpo<"sg", std::uint32_t, std::optional<unsigned>, std::string, unsigned, unsigned, char, char,
        double, double, double, double, std::string, std::vector<std::size_t>>;
    inline constexpr def_sg_cpo def_sg;

    /**
     * @typedef def_sg_mux_cpo
     * @brief Custom property object for the multiplexed signal definition in a DBC file.
     *
     * This type alias defines a custom property object (CPO) named "sg_mux" used to represent the multiplexed
     * signal information in a DBC file.
     */
    using def_sg_mux_cpo = cpo<"sg_mux", std::uint32_t, std::string, unsigned, unsigned, char, char, std::string,
        std::vector<std::size_t>>;
    inline constexpr def_sg_mux_cpo def_sg_mux;

    /**
     * @typedef def_ev_cpo
     * @brief Custom property object for the event definition in a DBC file.
     *
     * This type alias defines a custom property object (CPO) named "ev" used to represent the event
     * information in a DBC file.
     */
    using def_ev_cpo = cpo<"ev", std::string, unsigned, double, double, std::string, double, unsigned, std::string,
        std::vector<std::size_t>>;
    inline constexpr def_ev_cpo def_ev;

    /**
     * @typedef def_envvar_data_cpo
     * @brief Custom property object for the environment variable data definition in a DBC file.
     *
     * This type alias defines a custom property object (CPO) named "envvar_data" used to represent the
     * environment variable data information in a DBC file.
     */
    using def_envvar_data_cpo = cpo<"envvar_data", std::string, unsigned>;
    inline constexpr def_envvar_data_cpo def_envvar_data;

    /**
     * @typedef def_sgtype_ref_cpo
     * @brief Custom property object for the signal type reference definition in a DBC file.
     *
     * This type alias defines a custom property object (CPO) named "sgtype_ref" used to represent the signal
     * type reference information in a DBC file.
     */
    using def_sgtype_ref_cpo = cpo<"sgtype_ref", unsigned, std::string, std::string>;
    inline constexpr def_sgtype_ref_cpo def_sgtype_ref;

    /**
     * @typedef def_sgtype_cpo
     * @brief Custom property object for the signal type definition in a DBC file.
     *
     * This type alias defines a custom property object (CPO) named "sgtype" used to represent the signal
     * type information in a DBC file.
     */
    using def_sgtype_cpo = cpo<"sgtype", std::string, unsigned, char, char, double, double, double, double, std::string,
        double, std::size_t>;
    inline constexpr def_sgtype_cpo def_sgtype;

    /**
     * @typedef def_sig_group_cpo
     * @brief Custom property object for the signal group definition in a DBC file.
     *
     * This type alias defines a custom property object (CPO) named "sig_group" used to represent the signal
     * group information in a DBC file.
     */
    using def_sig_group_cpo = cpo<"sig_group", unsigned, std::string, unsigned, std::vector<std::string>>;
    inline constexpr def_sig_group_cpo def_sig_group;

    /**
     * @typedef def_cm_glob_cpo
     * @brief Custom property object for the global comment definition in a DBC file.
     *
     * This type alias defines a custom property object (CPO) named "cm" used to represent the global comment
     * information in a DBC file.
     */
    using def_cm_glob_cpo = cpo<"cm", std::string>;
    inline constexpr def_cm_glob_cpo def_cm_glob;

    /**
     * @typedef def_cm_bu_cpo
     * @brief Custom property object for the bus unit comment definition in a DBC file.
     *
     * This type alias defines a custom property object (CPO) named "cm_msg" used to represent the bus unit
     * comment information in a DBC file.
     */
    using def_cm_bu_cpo = cpo<"cm_msg", unsigned, std::string>;
    inline constexpr def_cm_bu_cpo def_cm_bu;

    /**
     * @typedef def_cm_bo_cpo
     * @brief Custom property object for the board comment definition in a DBC file.
     *
     * This type alias defines a custom property object (CPO) named "cm_bo" used to represent the board
     * comment information in a DBC file.
     */
    using def_cm_bo_cpo = cpo<"cm_bo", std::uint32_t, std::string>;
    inline constexpr def_cm_bo_cpo def_cm_bo;

    /**
     * @typedef def_cm_sg_cpo
     * @brief Custom property object for the signal comment definition in a DBC file.
     *
     * This type alias defines a custom property object (CPO) named "cm_sg" used to represent the signal
     * comment information in a DBC file.
     */
    using def_cm_sg_cpo = cpo<"cm_sg", std::uint32_t, std::string, std::string>;
	inline constexpr def_cm_sg_cpo def_cm_sg;

	/**
	 * @typedef def_cm_ev_cpo
	 * @brief Custom property object for the event comment definition in a DBC file.
	 *
	 * This type alias defines a custom property object (CPO) named "cm_ev" used to represent the event
	 * comment information in a DBC file.
	 */
	using def_cm_ev_cpo = cpo<"cm_ev", std::string, std::string>;
	inline constexpr def_cm_ev_cpo def_cm_ev;

	/**
	 * @typedef def_ba_def_enum_cpo
	 * @brief Custom property object for the enumeration attribute definition in a DBC file.
	 *
	 * This type alias defines a custom property object (CPO) named "ba_def_enum" used to represent the
	 * enumeration attribute information in a DBC file.
	 */
	using def_ba_def_enum_cpo = cpo<"ba_def_enum", std::string, std::string, std::vector<std::string>>;
	inline constexpr def_ba_def_enum_cpo def_ba_def_enum;

	/**
	 * @typedef def_ba_def_int_cpo
	 * @brief Custom property object for the integer attribute definition in a DBC file.
	 *
	 * This type alias defines a custom property object (CPO) named "ba_def_int" used to represent the
	 * integer attribute information in a DBC file.
	 */
	using def_ba_def_int_cpo = cpo<"ba_def_int", std::string, std::string, std::int32_t, std::int32_t>;
	inline constexpr def_ba_def_int_cpo def_ba_def_int;

	/**
	 * @typedef def_ba_def_float_cpo
	 * @brief Custom property object for the float attribute definition in a DBC file.
	 *
	 * This type alias defines a custom property object (CPO) named "ba_def_float" used to represent the
	 * float attribute information in a DBC file.
	 */
	using def_ba_def_float_cpo = cpo<"ba_def_float", std::string, std::string, double, double>;
	inline constexpr def_ba_def_float_cpo def_ba_def_float;

	/**
	 * @typedef def_ba_def_string_cpo
	 * @brief Custom property object for the string attribute definition in a DBC file.
	 *
	 * This type alias defines a custom property object (CPO) named "ba_def_string" used to represent the
	 * string attribute information in a DBC file.
	 */
	using def_ba_def_string_cpo = cpo<"ba_def_string", std::string, std::string>;
	inline constexpr def_ba_def_string_cpo def_ba_def_string;

	/**
	 * @typedef def_ba_def_def_cpo
	 * @brief Custom property object for the default attribute definition in a DBC file.
	 *
	 * This type alias defines a custom property object (CPO) named "ba_def_def" used to represent the
	 * default attribute information in a DBC file.
	 */
	using def_ba_def_def_cpo = cpo<"ba_def_def", std::string, std::variant<std::int32_t, double, std::string>>;
	inline constexpr def_ba_def_def_cpo def_ba_def_def;

	/**
	 * @typedef def_ba_cpo
	 * @brief Custom property object for the attribute definition in a DBC file.
	 *
	 * This type alias defines a custom property object (CPO) named "ba" used to represent the attribute
	 * information in a DBC file.
	 */
	using def_ba_cpo = cpo<"ba", std::string, std::string, std::string, std::size_t, unsigned,
		std::variant<std::int32_t, double, std::string>>;
	inline constexpr def_ba_cpo def_ba;

	/**
	 * @typedef def_val_env_cpo
	 * @brief Custom property object for the environment variable value definition in a DBC file.
	 *
	 * This type alias defines a custom property object (CPO) named "val_env" used to represent the environment
	 * variable value information in a DBC file.
	 */
	using def_val_env_cpo = cpo<"val_env", std::string, std::vector<std::pair<unsigned, std::string>>>;
	inline constexpr def_val_env_cpo def_val_env;

	/**
	 * @typedef def_val_sg_cpo
	 * @brief Custom property object for the signal value definition in a DBC file.
	 *
	 * This type alias defines a custom property object (CPO) named "val_sg" used to represent the signal
	 * value information in a DBC file.
	 */
	using def_val_sg_cpo = cpo<"val_sg", std::uint32_t, std::string, std::vector<std::pair<unsigned, std::string>>>;
	inline constexpr def_val_sg_cpo def_val_sg;

	/**
	 * @typedef def_val_table_cpo
	 * @brief Custom property object for the value table definition in a DBC file.
	 *
	 * This type alias defines a custom property object (CPO) named "val_table" used to represent the value
	 * table information in a DBC file.
	 */
	using def_val_table_cpo = cpo<"val_table", std::string, std::vector<std::pair<unsigned, std::string>>>;
	inline constexpr def_val_table_cpo def_val_table;

	/**
	 * @typedef def_sig_valtype_cpo
	 * @brief Custom property object for the signal value type definition in a DBC file.
	 *
	 * This type alias defines a custom property object (CPO) named "sig_valtype" used to represent the signal
	 * value type information in a DBC file.
	 */
	using def_sig_valtype_cpo = cpo<"sig_valtype", unsigned, std::string, unsigned>;
	inline constexpr def_sig_valtype_cpo def_sig_valtype;

	/**
	 * @typedef def_bo_tx_bu_cpo
	 * @brief Custom property object for the board transmit bus unit definition in a DBC file.
	 *
	 * This type alias defines a custom property object (CPO) named "bo_tx_bu" used to represent the board
	 * transmit bus unit information in a DBC file.
	 */
	using def_bo_tx_bu_cpo = cpo<"bo_tx_bu", unsigned, std::vector<std::string>>;
	inline constexpr def_bo_tx_bu_cpo def_bo_tx_bu;

	/**
	 * @typedef def_sg_mul_val_cpo
	 * @brief Custom property object for the signal multiple value definition in a DBC file.
	 *
	 * This type alias defines a custom property object (CPO) named "sg_mul_val" used to represent the signal
	 * multiple value information in a DBC file.
	 */
	using def_sg_mul_val_cpo = cpo<"sg_mul_val", unsigned, std::string, std::string, std::vector<std::pair<unsigned, unsigned>>>;
	inline constexpr def_sg_mul_val_cpo def_sg_mul_val;

    /**
     * @typedef interpreter
     * @brief A type alias for the mireo::any template instantiation with various CAN signal definitions.
     *
     * This type alias represents a collection of different CAN signal definitions that can be parsed by the DBC parser.
     * The definitions include:
     * - def_version: Definition for the version.
     * - def_bu: Definition for the bus unit.
     * - def_bo: Definition for the board.
     * - def_ev: Definition for the event.
     * - def_envvar_data: Definition for the environment variable data.
     * - def_sgtype: Definition for the signal type.
     * - def_sgtype_ref: Definition for the signal type reference.
     * - def_sig_group: Definition for the signal group.
     * - def_cm_glob: Definition for the global comment.
     * - def_cm_bu: Definition for the bus unit comment.
     * - def_cm_bo: Definition for the board comment.
     * - def_cm_sg: Definition for the signal comment.
     * - def_cm_ev: Definition for the event comment.
     * - def_ba_def_enum: Definition for the enumeration attribute.
     * - def_ba_def_int: Definition for the integer attribute.
     * - def_ba_def_float: Definition for the float attribute.
     * - def_ba_def_string: Definition for the string attribute.
     * - def_ba_def_def: Definition for the default attribute.
     * - def_ba: Definition for the attribute.
     * - def_sg: Definition for the signal.
     * - def_sg_mux: Definition for the multiplexed signal.
     * - def_val_env: Definition for the environment variable value.
     * - def_val_sg: Definition for the signal value.
     * - def_val_table: Definition for the value table.
     * - def_sig_valtype: Definition for the signal value type.
     * - def_bo_tx_bu: Definition for the board transmit bus unit.
     * - def_sg_mul_val: Definition for the signal multiple value.
     */
    using interpreter = mireo::any<def_version, def_bu, def_bo, def_ev, def_envvar_data, def_sgtype, def_sgtype_ref,
        def_sig_group, def_cm_glob, def_cm_bu, def_cm_bo, def_cm_sg, def_cm_ev, def_ba_def_enum, def_ba_def_int,
        def_ba_def_float, def_ba_def_string, def_ba_def_def, def_ba, def_sg, def_sg_mux, def_val_env, def_val_sg,
        def_val_table, def_sig_valtype, def_bo_tx_bu, def_sg_mul_val>;

    /**
     * @brief Parses the entire DBC file.
     *
     * @param dbc_src The DBC file source.
     * @param ipt The interpreter.
     * @return true if parsing was successful, false otherwise.
     */
    bool parse_dbc(std::string_view dbc_src, interpreter ipt);

} // end namespace can
