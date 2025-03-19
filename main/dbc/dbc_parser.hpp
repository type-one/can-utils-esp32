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
 * The parser is designed to be highly efficient and strictly follows the grammar rules from the latest DBC specification.
 * It is adapted for the ESP32 micro-controller and is a fork and modification of the MIREO version.
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

    template <std::size_t N>
    struct method_name
    {
        constexpr method_name(const char (&d)[N])
        {
			(void)d;
        }
    };

    template <method_name method, typename... Args>
    struct cpo
    {
        using type_erased_signature_t = void(mireo::this_&, Args...);

        template <typename T>
        requires mireo::tag_invocable<cpo, T&, Args...>
        void operator()(T& x, Args... args) const
        {
            return mireo::tag_invoke(*this, x, std::forward<Args>(args)...);
        }

        template <typename T>
        friend void tag_invoke(cpo<method, Args...>, std::reference_wrapper<T>& t, Args... args)
        {
            mireo::tag_invoke(cpo<method, Args...> {}, t.get(), std::forward<Args>(args)...);
        }

        template <typename T>
        friend void tag_invoke(cpo<method, Args...>, T&, Args...)
        {
        }
    };

    using def_version_cpo = cpo<"version", std::string>;
    inline constexpr def_version_cpo def_version;

    using def_bu_cpo = cpo<"bu", std::vector<std::string>>;
    inline constexpr def_bu_cpo def_bu;

    using def_bo_cpo = cpo<"bo", std::uint32_t, std::string, std::size_t, std::size_t>;
    inline constexpr def_bo_cpo def_bo;

    using def_sg_cpo = cpo<"sg", std::uint32_t, std::optional<unsigned>, std::string, unsigned, unsigned, char, char, double,
        double, double, double, std::string, std::vector<std::size_t>>;
    inline constexpr def_sg_cpo def_sg;

    using def_sg_mux_cpo
        = cpo<"sg_mux", std::uint32_t, std::string, unsigned, unsigned, char, char, std::string, std::vector<std::size_t>>;
    inline constexpr def_sg_mux_cpo def_sg_mux;

    using def_ev_cpo = cpo<"ev", std::string, unsigned, double, double, std::string, double, unsigned, std::string,
        std::vector<std::size_t>>;
    inline constexpr def_ev_cpo def_ev;

    using def_envvar_data_cpo = cpo<"envvar_data", std::string, unsigned>;
    inline constexpr def_envvar_data_cpo def_envvar_data;

    using def_sgtype_ref_cpo = cpo<"sgtype_ref", unsigned, std::string, std::string>;
    inline constexpr def_sgtype_ref_cpo def_sgtype_ref;

    using def_sgtype_cpo
        = cpo<"sgtype", std::string, unsigned, char, char, double, double, double, double, std::string, double, std::size_t>;
    inline constexpr def_sgtype_cpo def_sgtype;

    using def_sig_group_cpo = cpo<"sig_group", unsigned, std::string, unsigned, std::vector<std::string>>;
    inline constexpr def_sig_group_cpo def_sig_group;

    using def_cm_glob_cpo = cpo<"cm", std::string>;
    inline constexpr def_cm_glob_cpo def_cm_glob;

    using def_cm_bu_cpo = cpo<"cm_msg", unsigned, std::string>;
    inline constexpr def_cm_bu_cpo def_cm_bu;

    using def_cm_bo_cpo = cpo<"cm_bo", std::uint32_t, std::string>;
    inline constexpr def_cm_bo_cpo def_cm_bo;

    using def_cm_sg_cpo = cpo<"cm_sg", std::uint32_t, std::string, std::string>;
    inline constexpr def_cm_sg_cpo def_cm_sg;

    using def_cm_ev_cpo = cpo<"cm_ev", std::string, std::string>;
    inline constexpr def_cm_ev_cpo def_cm_ev;

    using def_ba_def_enum_cpo = cpo<"ba_def_enum", std::string, std::string, std::vector<std::string>>;
    inline constexpr def_ba_def_enum_cpo def_ba_def_enum;

    using def_ba_def_int_cpo = cpo<"ba_def_int", std::string, std::string, std::int32_t, std::int32_t>;
    inline constexpr def_ba_def_int_cpo def_ba_def_int;

    using def_ba_def_float_cpo = cpo<"ba_def_float", std::string, std::string, double, double>;
    inline constexpr def_ba_def_float_cpo def_ba_def_float;

    using def_ba_def_string_cpo = cpo<"ba_def_string", std::string, std::string>;
    inline constexpr def_ba_def_string_cpo def_ba_def_string;

    using def_ba_def_def_cpo = cpo<"ba_def_def", std::string, std::variant<std::int32_t, double, std::string>>;
    inline constexpr def_ba_def_def_cpo def_ba_def_def;

    using def_ba_cpo = cpo<"ba", std::string, std::string, std::string, std::size_t, unsigned,
        std::variant<std::int32_t, double, std::string>>;
    inline constexpr def_ba_cpo def_ba;

    using def_val_env_cpo = cpo<"val_env", std::string, std::vector<std::pair<unsigned, std::string>>>;
    inline constexpr def_val_env_cpo def_val_env;

    using def_val_sg_cpo = cpo<"val_sg", std::uint32_t, std::string, std::vector<std::pair<unsigned, std::string>>>;
    inline constexpr def_val_sg_cpo def_val_sg;

    using def_val_table_cpo = cpo<"val_table", std::string, std::vector<std::pair<unsigned, std::string>>>;
    inline constexpr def_val_table_cpo def_val_table;

    using def_sig_valtype_cpo = cpo<"sig_valtype", unsigned, std::string, unsigned>;
    inline constexpr def_sig_valtype_cpo def_sig_valtype;

    using def_bo_tx_bu_cpo = cpo<"bo_tx_bu", unsigned, std::vector<std::string>>;
    inline constexpr def_bo_tx_bu_cpo def_bo_tx_bu;

    using def_sg_mul_val_cpo
        = cpo<"sg_mul_val", unsigned, std::string, std::string, std::vector<std::pair<unsigned, unsigned>>>;
    inline constexpr def_sg_mul_val_cpo def_sg_mul_val;

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
