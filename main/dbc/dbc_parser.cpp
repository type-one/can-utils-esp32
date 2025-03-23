/*==========================================================================================
    Copyright (c) 2001-2023 Mireo, EU

    Extremely efficient DBC file format parser, built on top of Boost Spirit.

    Strictly follows the grammar rules from the last updated DBC specification available at
    http://mcu.so/Microcontroller/Automotive/dbc-file-format-documentation_compress.pdf
===========================================================================================*/

/**
 * @file dbc_parser.cppp
 * @brief Implementation file - Extremely efficient DBC file format parser, built on top of Boost Spirit.
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

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>


#include "boost/fusion/adapted/std_pair.hpp"
#include "boost/spirit/home/x3.hpp"


#include "dbc_parser.hpp"

namespace x3 = boost::spirit::x3;

namespace can
{

    /**
     * @brief Converts the parsed attribute to the specified type.
     *
     * @tparam T The type to convert to.
     * @param arg The argument to convert.
     * @return A lambda function that performs the conversion.
     */
    template <typename T>
    constexpr auto to(T& arg)
    {
        return [&](auto& ctx) { arg = x3::_attr(ctx); };
    }

    /**
     * @brief Creates a parser rule for the specified type.
     *
     * @tparam T The type to create the rule for.
     * @tparam Parser The parser to use.
     * @param p The parser to use.
     * @return The created parser rule.
     */
    template <typename T, typename Parser>
    constexpr auto as(Parser&& p)
    {
        return x3::rule<struct _, T> {} = std::forward<Parser>(p);
    }

    /**
     * @brief Selects a parser based on the specified tag and symbols.
     *
     * @tparam Tag The tag to use.
     * @tparam Symbols The symbols to use.
     * @param sym The symbols to use.
     * @return The selected parser.
     */
    template <typename Tag, typename Symbols>
    constexpr auto select_parser(Symbols&& sym)
    {
        auto action = [](auto& ctx) { x3::get<Tag>(ctx) = x3::_attr(ctx); };
        return x3::omit[sym[action]];
    }

    /**
     * @brief A lazy parser type.
     *
     * @tparam Tag The tag to use.
     */
    template <typename Tag>
    struct lazy_type : x3::parser<lazy_type<Tag>>
    {
        using attribute_type = typename Tag::attribute_type;

        /**
         * @brief Parses the input using the lazy parser.
         *
         * @tparam It The iterator type.
         * @tparam Ctx The context type.
         * @tparam RCtx The rule context type.
         * @tparam Attr The attribute type.
         * @param first The beginning of the input.
         * @param last The end of the input.
         * @param ctx The context.
         * @param rctx The rule context.
         * @param attr The attribute.
         * @return true if parsing was successful, false otherwise.
         */
        template <typename It, typename Ctx, typename RCtx, typename Attr>
        bool parse(It& first, It last, Ctx& ctx, RCtx& rctx, Attr& attr) const
        {
            auto& subject = x3::get<Tag>(ctx);

            It saved = first;
            x3::skip_over(first, last, ctx);
            bool rv = x3::as_parser(subject).parse(first, last, std::forward<Ctx>(ctx), std::forward<RCtx>(rctx), attr);
            if (rv)
            {
                return true;
            }
            first = saved;
            return false;
        }
    };

    template <typename T>
    constexpr auto lazy = lazy_type<T> {};

    constexpr auto skipper_
        = x3::lexeme[x3::blank | "//" >> *(x3::char_ - x3::eol) | ("/*" >> *(x3::char_ - "*/")) >> "*/"];

    constexpr auto end_cmd_ = x3::omit[*x3::eol];

    constexpr auto od(char c)
    {
        return x3::omit[x3::char_(c)];
    }

    const auto name_ = as<std::string>(x3::lexeme[x3::char_("a-zA-Z_") >> *x3::char_("a-zA-Z_0-9")]);

    const auto quoted_name_
        = as<std::string>(x3::lexeme['"' >> *(('\\' >> x3::char_("\\\"")) | ~x3::char_('"')) >> '"']);

    /**
     * @brief Skips blank lines in the input.
     *
     * @param rng The input range.
     * @return The input range with blank lines skipped.
     */
    static std::string_view skip_blines(std::string_view rng)
    {
        constexpr auto eols_ = x3::omit[+x3::eol];
        auto iter = rng.begin();
        phrase_parse(iter, rng.end(), eols_, skipper_);
        return { iter, rng.end() };
    }

    using parse_rv = std::pair<std::string_view, bool>;

    using nodes_t = x3::symbols<std::size_t>;

    using attr_val_t = std::variant<std::int32_t, double, std::string>;
    using attr_val_rule = x3::any_parser<std::string_view::const_iterator, attr_val_t>;
    using attr_types_t = x3::symbols<attr_val_rule>;

    /**
     * @brief Creates a parse result value.
     *
     * @param b The beginning of the input.
     * @param e The end of the input.
     * @param v The parse result value.
     * @return The created parse result value.
     */
    static auto make_rv(std::string_view::iterator b, std::string_view::iterator e, bool v)
    {
        return std::make_pair(std::string_view { b, e }, v);
    }

    /**
     * @brief Creates a parse result value.
     *
     * @param s The input range.
     * @param v The parse result value.
     * @return The created parse result value.
     */
    static auto make_rv(std::string_view s, bool v)
    {
        return std::make_pair(s, v);
    }

    /**
     * @brief Parses the version section of the DBC file.
     *
     * @param rng The input range.
     * @param ipt The interpreter.
     * @return The parse result value.
     */
    static const parse_rv parse_version(std::string_view rng, interpreter& ipt)
    {
        bool has_sect = false;
        std::string version;

        const auto version_ = x3::omit[x3::lexeme[x3::lit("VERSION") >> +x3::blank]] >> x3::attr(true)[to(has_sect)]
            >> quoted_name_[to(version)] >> end_cmd_;

        auto iter = rng.begin();
        if (!phrase_parse(iter, rng.end(), version_, skipper_))
        {
            return make_rv(rng, !has_sect);
        }

        def_version(ipt, std::move(version));
        return make_rv(iter, rng.end(), true);
    };

    /**
     * @brief Parses the NS_ section of the DBC file.
     *
     * @param rng The input range.
     * @return The parse result value.
     *
     * @note The NS_ section in a DBC file defines the new symbols used in the file. It is a list of keywords that
     *       describe various attributes and elements in the DBC file, such as signal types, value tables, and other
     *       custom definitions.
     */
    static const parse_rv parse_ns_(std::string_view rng)
    {
        struct ns_syms : x3::symbols<unsigned>
        {
            ns_syms()
            {
                add("NS_DESC_", 0)("CM_", 1)("BA_DEF_", 2)("BA_", 3)("VAL_", 4)("CAT_DEF_", 5)("CAT_", 6)("FILTER", 7)(
                    "BA_DEF_DEF_", 8)("EV_DATA_", 9)("ENVVAR_DATA_", 10)("SGTYPE_", 11)("SGTYPE_VAL_", 12)(
                    "BA_DEF_SGTYPE_", 13)("BA_SGTYPE_", 14)("SIG_TYPE_REF_", 15)("VAL_TABLE_", 16)("SIG_GROUP_", 17)(
                    "SIG_VALTYPE_", 18)("SIGTYPE_VALTYPE_", 19)("BO_TX_BU_", 20)("BA_DEF_REL_", 21)("BA_REL_", 22)(
                    "BA_DEF_DEF_REL_", 23)("BU_SG_REL_", 24)("BU_EV_REL_", 25)("BU_BO_REL_", 26)("SG_MUL_VAL_", 27);
            }
        } ns_syms_;
        bool has_sect = false;

        const auto ns_ = x3::omit[x3::lexeme[x3::lit("NS_") >> +x3::blank >> ':']] >> x3::attr(true)[to(has_sect)]
            >> x3::omit[+x3::space] >> x3::omit[*(ns_syms_ >> x3::omit[+x3::space])] >> end_cmd_;

        auto iter = rng.begin();
        if (!phrase_parse(iter, rng.end(), ns_, skipper_))
        {
            return make_rv(rng, !has_sect);
        }

        // interpreter callback deliberately omitted
        return make_rv(iter, rng.end(), true);
    }

    /**
     * @brief Parses the BS_ section of the DBC file.
     *
     * @param rng The input range.
     * @return The parse result value.
     *
     * @note The BS_ section in a DBC file defines the bit timing parameters for the CAN network.
     *       It specifies the baud rate and other related parameters that are used to configure
     *       the CAN controllers on the network.
     */
    static const parse_rv parse_bs_(std::string_view rng)
    {
        const auto bs_ = x3::omit[x3::lexeme[x3::lit("BS_:")]]
            >> x3::omit[-(x3::uint_ >> ':' >> x3::uint_ >> ',' >> x3::uint_)] >> end_cmd_;

        auto iter = rng.begin();
        if (!phrase_parse(iter, rng.end(), bs_, skipper_))
        {
            return make_rv(rng, false);
        }

        // interpreter callback deliberately omitted
        return make_rv(iter, rng.end(), true);
    };

    /**
     * @brief Parses the BU_ section of the DBC file.
     *
     * @param rng The input range.
     * @param nodes The nodes.
     * @param ipt The interpreter.
     * @return The parse result value.
     *
     * @note The BU_ section in a DBC file defines the nodes (ECUs) in the CAN network.
     *       It lists the names of all the nodes that are part of the network.
     */
    static const parse_rv parse_bu_(std::string_view rng, nodes_t& nodes, interpreter& ipt)
    {
        std::vector<std::string> node_names;

        const auto bu_ = x3::omit[x3::lexeme[x3::lit("BU_:")]] >> (*name_)[to(node_names)] >> end_cmd_;

        auto iter = rng.begin();
        if (!phrase_parse(iter, rng.end(), bu_, skipper_))
        {
            return make_rv(rng, false);
        }

        unsigned node_ord = 0;
        for (const auto& nn : node_names)
        {
            nodes.add(nn, node_ord++);
        }
        nodes.add("Vector__XXX", node_ord);

        def_bu(ipt, std::move(node_names));
        return make_rv(iter, rng.end(), true);
    };

    /**
     * @brief Parses the SG_ section of the DBC file.
     *
     * @param rng The input range.
     * @param nodes The nodes.
     * @param ipt The interpreter.
     * @param can_id The CAN ID.
     * @return The parse result value.
     *
     * @note The SG_ section in a DBC file defines the signals within a message. Each signal represents a piece of data
     *       transmitted in the message, including its name, start bit, size, byte order, value type, factor, offset,
     *       minimum and maximum values, unit, and the nodes that receive the signal.
     */
    static const parse_rv parse_sg_(std::string_view rng, const nodes_t& nodes, interpreter& ipt, std::uint32_t can_id)
    {
        bool has_sect = false;
        std::optional<unsigned> sg_mux_switch_val;
        std::string sg_name;
        std::optional<char> sg_mux_switch;
        unsigned sg_start_bit, sg_size;
        char sg_byte_order, sg_sign;
        double sg_factor, sg_offset;
        double sg_min, sg_max;
        std::string sg_unit;
        std::vector<std::size_t> rec_ords;

        const auto mux_ = -(x3::char_('m') >> x3::uint_[to(sg_mux_switch_val)]) >> -x3::char_('M')[to(sg_mux_switch)];

        const auto sg_ = x3::omit[x3::lexeme[x3::lit("SG_") >> +x3::blank]] >> x3::attr(true)[to(has_sect)]
            >> name_[to(sg_name)] >> mux_ >> od(':') >> x3::uint_[to(sg_start_bit)] >> od('|') >> x3::uint_[to(sg_size)]
            >> od('@') >> as<char>(x3::char_('0') | x3::char_('1'))[to(sg_byte_order)]
            >> as<char>(x3::char_('+') | x3::char_('-'))[to(sg_sign)] >> od('(') >> x3::double_[to(sg_factor)]
            >> od(',') >> x3::double_[to(sg_offset)] >> od(')') >> od('[') >> x3::double_[to(sg_min)] >> od('|')
            >> x3::double_[to(sg_max)] >> od(']') >> quoted_name_[to(sg_unit)] >> (nodes % ',')[to(rec_ords)]
            >> end_cmd_;

        for (auto iter = rng.begin(); iter != rng.end(); has_sect = false)
        {
            sg_mux_switch_val.reset();
            sg_mux_switch.reset();

            if (!phrase_parse(iter, rng.end(), sg_, skipper_)
                || std::abs(sg_factor) <= std::numeric_limits<double>::epsilon())
            {
                return make_rv(iter, rng.end(), !has_sect);
            }

            if (sg_mux_switch.value_or(' ') == 'M')
            {
                def_sg_mux(ipt, can_id, sg_name, sg_start_bit, sg_size, sg_byte_order, sg_sign, std::move(sg_unit),
                    std::move(rec_ords));
            }
            else
            {
                def_sg(ipt, can_id, sg_mux_switch_val, sg_name, sg_start_bit, sg_size, sg_byte_order, sg_sign,
                    sg_factor, sg_offset, sg_min, sg_max, std::move(sg_unit), std::move(rec_ords));
            }
        }
        return make_rv(rng.end(), rng.end(), true);
    }


    /**
     * @brief Parses the BO_ section of the DBC file.
     *
     * @param rng The input range.
     * @param nodes The nodes.
     * @param ipt The interpreter.
     * @return The parse result value.
     *
     * @note The BO_ section in a DBC file defines the messages (frames) in the CAN network.
     *       It specifies the message ID, message name, message size, and the transmitter node.
     *       Each message can contain multiple signals (SG_) that represent the data transmitted
     *       within the message.
     */
    static const parse_rv parse_bo_(std::string_view rng, const nodes_t& nodes, interpreter& ipt)
    {
        bool has_sect = false;
        std::uint32_t can_id;
        std::string msg_name;
        std::size_t msg_size;
        std::size_t transmitter_ord;

        const auto bo_ = x3::omit[x3::lexeme[x3::lit("BO_") >> +x3::blank]] >> x3::attr(true)[to(has_sect)]
            >> x3::uint_[to(can_id)] >> name_[to(msg_name)] >> od(':') >> x3::uint_[to(msg_size)]
            >> nodes[to(transmitter_ord)] >> end_cmd_;

        for (auto iter = rng.begin(); iter != rng.end(); has_sect = false)
        {
            if (!phrase_parse(iter, rng.end(), bo_, skipper_))
            {
                return make_rv(iter, rng.end(), !has_sect);
            }

            def_bo(ipt, can_id, std::move(msg_name), msg_size, transmitter_ord);

            auto [remain_rng, expected] = parse_sg_({ iter, rng.end() }, nodes, ipt, can_id);
            if (!expected)
            {
                return make_rv(remain_rng, false);
            }
            iter = remain_rng.begin();
        }

        return make_rv(rng.end(), rng.end(), true);
    }

    /**
     * @brief Parses the EV_ section of the DBC file.
     *
     * @param rng The input range.
     * @param nodes The nodes.
     * @param ipt The interpreter.
     * @return The parse result value.
     *
     * @note The EV_ section in a DBC file defines the environment variables used in the CAN network.
     *       Environment variables are used to store values that can be shared across different nodes
     *       and messages in the network. They can represent various types of data, such as counters,
     *       timers, or configuration parameters.
     */
    static const parse_rv parse_ev_(std::string_view rng, const nodes_t& nodes, interpreter& ipt)
    {
        bool has_sect = false;
        std::string ev_name;
        unsigned ev_type;
        double ev_min, ev_max;
        std::string ev_unit;
        double ev_initial;
        unsigned ev_id;
        std::string ev_access_type;
        std::vector<size_t> ev_access_nodes_ords;

        const auto access_type_ = x3::lexeme[x3::lit("DUMMY_NODE_VECTOR") >> -x3::lit("800") >> x3::char_("0-3")];

        const auto ev_ = x3::omit[x3::lexeme[x3::lit("EV_") >> +x3::blank]] >> x3::attr(true)[to(has_sect)]
            >> name_[to(ev_name)] >> od(':') >> &x3::char_("0-2") >> x3::uint_[to(ev_type)] >> od('[')
            >> x3::double_[to(ev_min)] >> od('|') >> x3::double_[to(ev_max)] >> od(']') >> quoted_name_[to(ev_unit)]
            >> x3::double_[to(ev_initial)] >> x3::uint_[to(ev_id)] >> as<std::string>(access_type_)[to(ev_access_type)]
            >> (nodes % ',')[to(ev_access_nodes_ords)] >> od(';') >> end_cmd_;

        for (auto iter = rng.begin(); iter != rng.end(); has_sect = false)
        {
            if (!phrase_parse(iter, rng.end(), ev_, skipper_))
            {
                return make_rv(iter, rng.end(), !has_sect);
            }

            def_ev(ipt, std::move(ev_name), ev_type, ev_min, ev_max, std::move(ev_unit), ev_initial, ev_id,
                std::move(ev_access_type), std::move(ev_access_nodes_ords));
        }
        return make_rv(rng.end(), rng.end(), true);
    }

    /**
     * @brief Parses the ENVVAR_DATA_ section of the DBC file.
     *
     * @param rng The input range.
     * @param ipt The interpreter.
     * @return The parse result value.
     *
     * @note The ENVVAR_DATA_ section in a DBC file defines the size of environment variables.
     *       Environment variables are used to store values that can be shared across different nodes
     *       and messages in the network. This section specifies the size of the data associated with
     *       each environment variable.
     */
    static const parse_rv parse_envvar_data_(std::string_view rng, interpreter& ipt)
    {
        bool has_sect = false;
        std::string ev_name;
        unsigned data_size;

        const auto envar_data_ = x3::omit[x3::lexeme[x3::lit("ENVVAR_DATA_") >> +x3::blank]]
            >> x3::attr(true)[to(has_sect)] >> name_[to(ev_name)] >> od(':') >> x3::uint_[to(data_size)] >> od(';')
            >> end_cmd_;

        for (auto iter = rng.begin(); iter != rng.end(); has_sect = false)
        {
            if (!phrase_parse(iter, rng.end(), envar_data_, skipper_))
            {
                return make_rv(iter, rng.end(), !has_sect);
            }

            def_envvar_data(ipt, std::move(ev_name), data_size);
        }

        return make_rv(rng.end(), rng.end(), true);
    }

    /**
     * @brief Parses the SGTYPE_ section of the DBC file.
     *
     * @param rng The input range.
     * @param val_tables The value tables.
     * @param ipt The interpreter.
     * @return The parse result value.
     *
     * @note The SGTYPE_ section in a DBC file defines signal types. It specifies the characteristics of signals,
     *       such as size, byte order, sign, factor, offset, minimum and maximum values, unit, default value, and
     *       value table. This section can also reference existing signal types for specific messages and signals.
     */
    static const parse_rv parse_sgtype_(std::string_view rng, const nodes_t& val_tables, interpreter& ipt)
    {
        bool has_sect = false;
        std::optional<unsigned> msg_id;
        std::string sg_name, sg_type_name;
        unsigned sg_size;
        char sg_byte_order, sg_sign;
        double sg_factor, sg_offset;
        double sg_min, sg_max;
        std::string sg_unit;
        double sg_default_val;
        std::size_t val_table_ord;

        const auto sg_type_t_ = name_[to(sg_type_name)] >> od(':') >> x3::uint_[to(sg_size)] >> od('@')
            >> as<char>(x3::char_('0') | x3::char_('1'))[to(sg_byte_order)]
            >> as<char>(x3::char_('+') | x3::char_('-'))[to(sg_sign)] >> od('(') >> x3::double_[to(sg_factor)]
            >> od(',') >> x3::double_[to(sg_offset)] >> od(')') >> od('[') >> x3::double_[to(sg_min)] >> od('|')
            >> x3::double_[to(sg_max)] >> od(']') >> quoted_name_[to(sg_unit)] >> x3::double_[to(sg_default_val)]
            >> od(',') >> val_tables[to(val_table_ord)];

        const auto sg_type_ref_ = x3::uint_[to(msg_id)] >> name_[to(sg_name)] >> od(':') >> name_[to(sg_type_name)];

        const auto sg_type_ = x3::omit[x3::lexeme[x3::lit("SGTYPE_") >> +x3::blank]] >> x3::attr(true)[to(has_sect)]
            >> (sg_type_t_ | sg_type_ref_) >> od(';') >> end_cmd_;

        for (auto iter = rng.begin(); iter != rng.end(); has_sect = false)
        {
            msg_id.reset();

            if (!phrase_parse(iter, rng.end(), sg_type_, skipper_))
            {
                return make_rv(iter, rng.end(), !has_sect);
            }

            if (msg_id.has_value())
            {
                def_sgtype_ref(ipt, msg_id.value(), std::move(sg_name), std::move(sg_type_name));
            }
            else
            {
                def_sgtype(ipt, std::move(sg_type_name), sg_size, sg_byte_order, sg_sign, sg_factor, sg_offset, sg_min,
                    sg_max, std::move(sg_unit), sg_default_val, val_table_ord);
            }
        }
        return make_rv(rng.end(), rng.end(), true);
    }

    /**
     * @brief Parses the SIG_GROUP_ section of the DBC file.
     *
     * @param rng The input range.
     * @param ipt The interpreter.
     * @return The parse result value.
     *
     * @note The SIG_GROUP_ section in a DBC file defines a group of signals that are transmitted together.
     *       It specifies the message ID, signal group name, the number of repetitions, and the list of signal names
     *       that belong to the group. This section is used to group related signals for easier management and
     *       interpretation.
     */
    static const parse_rv parse_sig_group_(std::string_view rng, interpreter& ipt)
    {
        bool has_sect = false;
        unsigned msg_id;
        std::string sig_group_name;
        unsigned repetitions;
        std::vector<std::string> sig_names;

        const auto sig_group_ = x3::omit[x3::lexeme[x3::lit("SIG_GROUP_") >> +x3::blank]]
            >> x3::attr(true)[to(has_sect)] >> x3::uint_[to(msg_id)] >> name_[to(sig_group_name)]
            >> x3::uint_[to(repetitions)] >> od(':') >> (name_ % ',')[to(sig_names)] >> od(';') >> end_cmd_;

        for (auto iter = rng.begin(); iter != rng.end(); has_sect = false)
        {
            if (!phrase_parse(iter, rng.end(), sig_group_, skipper_))
            {
                return make_rv(iter, rng.end(), !has_sect);
            }

            def_sig_group(ipt, msg_id, std::move(sig_group_name), repetitions, std::move(sig_names));
        }
        return make_rv(rng.end(), rng.end(), true);
    }

    /**
     * @brief Parses the CM_ section of the DBC file.
     *
     * @param rng The input range.
     * @param nodes The nodes.
     * @param ipt The interpreter.
     * @return The parse result value.
     *
     * @note The CM_ section in a DBC file is used to add comments to various elements within the file.
     *       These comments can be associated with messages, signals, nodes, or environment variables.
     *       The comments provide additional information or documentation for the elements they are associated with.
     */
    static const parse_rv parse_cm_(std::string_view rng, const nodes_t& nodes, interpreter& ipt)
    {
        bool has_sect = false;
        std::string object_type, comment_text;

        unsigned bu_ord;
        std::uint32_t message_id;
        std::string object_name;

        const auto comment_
            = as<std::string>(x3::lexeme['"' >> *(('\\' >> x3::char_("\\\"")) | ~x3::char_('"') | x3::eol) >> '"']);

        const auto cm_glob_ = comment_[to(comment_text)];
        const auto cm_bu_ = x3::lexeme[x3::string("BU_")[to(object_type)] >> +x3::blank] >> nodes[to(bu_ord)]
            >> comment_[to(comment_text)];
        const auto cm_sg_ = x3::lexeme[x3::string("SG_")[to(object_type)] >> +x3::blank] >> x3::uint_[to(message_id)]
            >> name_[to(object_name)] >> comment_[to(comment_text)];
        const auto cm_bo_ = x3::lexeme[x3::string("BO_")[to(object_type)] >> +x3::blank] >> x3::uint_[to(message_id)]
            >> comment_[to(comment_text)];
        const auto cm_ev_ = x3::lexeme[x3::string("EV_")[to(object_type)] >> +x3::blank] >> name_[to(object_name)]
            >> comment_[to(comment_text)];

        const auto cm_ = x3::omit[x3::lexeme[x3::lit("CM_") >> +x3::blank]] >> x3::attr(true)[to(has_sect)]
            >> (cm_sg_ | cm_bo_ | cm_bu_ | cm_ev_ | cm_glob_) >> od(';') >> end_cmd_;

        for (auto iter = rng.begin(); iter != rng.end(); has_sect = false)
        {
            object_type.clear();

            if (!phrase_parse(iter, rng.end(), cm_, skipper_))
            {
                return make_rv(iter, rng.end(), !has_sect);
            }

            if (object_type.empty())
            {
                def_cm_glob(ipt, std::move(comment_text));
            }
            else if (object_type == "BU_")
            {
                def_cm_bu(ipt, bu_ord, std::move(comment_text));
            }
            else if (object_type == "BO_")
            {
                def_cm_bo(ipt, message_id, std::move(comment_text));
            }
            else if (object_type == "SG_")
            {
                def_cm_sg(ipt, message_id, std::move(object_name), std::move(comment_text));
            }
            else if (object_type == "EV_")
            {
                def_cm_ev(ipt, std::move(object_name), std::move(comment_text));
            }
        }
        return make_rv(rng.end(), rng.end(), true);
    }

    /**
     * @brief Parses the BA_DEF_ section of the DBC file.
     *
     * @param rng The input range.
     * @param ats_ The attribute types.
     * @param ipt The interpreter.
     * @return The parse result value.
     *
     * @note The BA_DEF_ section in a DBC file defines attribute definitions.
     *       It specifies the name, type, and range of attributes that can be associated with various elements
     *       in the DBC file, such as nodes, messages, signals, and environment variables.
     *       These attributes provide additional metadata and configuration options for the elements they are associated
     * with.
     */
    static const parse_rv parse_ba_def_(std::string_view rng, attr_types_t& ats_, interpreter& ipt)
    {
        bool has_sect = false;
        std::string object_type, attr_name, data_type;
        std::int32_t int_min, int_max;
        double dbl_min, dbl_max;
        std::vector<std::string> enum_vals;

        const auto obj_type_
            = as<std::string>(x3::lexeme[(x3::string("BU_") | x3::string("BO_") | x3::string("SG_") | x3::string("EV_"))
                >> x3::omit[+x3::blank]]);

        const auto att_hexint_ = as<std::string>(x3::lexeme[x3::string("HEX") | x3::string("INT")])[to(data_type)]
            >> x3::int_[to(int_min)] >> x3::int_[to(int_max)];

        const auto att_float_ = as<std::string>(x3::lexeme[x3::lit("FLOAT")])[to(data_type)] >> x3::double_[to(dbl_min)]
            >> x3::double_[to(dbl_max)];

        const auto att_str_ = as<std::string>(x3::lexeme[x3::lit("STRING")])[to(data_type)];

        const auto att_enum_
            = as<std::string>(x3::lexeme[x3::lit("ENUM")])[to(data_type)] >> -(quoted_name_ % ',')[to(enum_vals)];


        const auto ba_def_ = x3::omit[x3::lexeme[x3::lit("BA_DEF_") >> +x3::blank]] >> x3::attr(true)[to(has_sect)]
            >> -obj_type_[to(object_type)] >> quoted_name_[to(attr_name)]
            >> (att_hexint_ | att_float_ | att_str_ | att_enum_) >> od(';') >> end_cmd_;

        for (auto iter = rng.begin(); iter != rng.end(); has_sect = false)
        {
            object_type.clear();
            if (!phrase_parse(iter, rng.end(), ba_def_, skipper_))
            {
                return make_rv(iter, rng.end(), !has_sect);
            }

            if (data_type == "ENUM")
            {
                ats_.add("\"" + attr_name + "\"", quoted_name_ | x3::int_);
                def_ba_def_enum(ipt, std::move(attr_name), std::move(object_type), std::move(enum_vals));
            }
            else if (data_type == "INT" || data_type == "HEX")
            {
                ats_.add("\"" + attr_name + "\"", x3::int_);
                def_ba_def_int(ipt, std::move(attr_name), std::move(object_type), int_min, int_max);
            }
            else if (data_type == "FLOAT")
            {
                ats_.add("\"" + attr_name + "\"", x3::double_);
                def_ba_def_float(ipt, std::move(attr_name), std::move(object_type), dbl_min, dbl_max);
            }
            else if (data_type == "STRING")
            {
                ats_.add("\"" + attr_name + "\"", quoted_name_);
                def_ba_def_string(ipt, std::move(attr_name), std::move(object_type));
            }
        }
        return make_rv(rng.end(), rng.end(), true);
    }

    /**
     * @brief Parses the BA_DEF_DEF_ section of the DBC file.
     *
     * @param rng The input range.
     * @param ats_ The attribute types.
     * @param ipt The interpreter.
     * @return The parse result value.
     *
     * @note The BA_DEF_DEF_ section in a DBC file defines the default values for attributes.
     *       It specifies the default value for each attribute defined in the BA_DEF_ section.
     *       These default values are used when no specific value is provided for an attribute
     *       in the DBC file. This section helps in maintaining consistent attribute values
     *       across different elements in the DBC file.
     */
    static const parse_rv parse_ba_def_def_(std::string_view rng, const attr_types_t& ats_, interpreter& ipt)
    {
        bool has_sect = false;
        std::string attr_name;
        attr_val_t attr_val;

        const auto val_ = x3::with<attr_val_rule>(attr_val_rule {})[&select_parser<attr_val_rule>(ats_)
            >> quoted_name_[to(attr_name)] >> lazy<attr_val_rule>[to(attr_val)]];

        const auto ba_def_def_ = x3::with<attr_val_rule>(attr_val_rule {})
            [x3::omit[x3::lexeme[(x3::string("BA_DEF_DEF_REL_") | x3::string("BA_DEF_DEF_")) >> +x3::blank]]
                >> x3::attr(true)[to(has_sect)] >> val_ >> od(';') >> end_cmd_];

        for (auto iter = rng.begin(); iter != rng.end(); has_sect = false)
        {
            if (!phrase_parse(iter, rng.end(), ba_def_def_, skipper_))
            {
                return make_rv(iter, rng.end(), !has_sect);
            }

            def_ba_def_def(ipt, std::move(attr_name), std::move(attr_val));
        }
        return make_rv(rng.end(), rng.end(), true);
    }

    /**
     * @brief Parses the BA_ section of the DBC file.
     *
     * @param rng The input range.
     * @param nodes The nodes.
     * @param ats_ The attribute types.
     * @param ipt The interpreter.
     * @return The parse result value.
     *
     * @note The BA_ section in a DBC file defines attribute values for various elements such as nodes, messages,
     * signals, and environment variables. These attributes provide additional metadata and configuration options for
     * the elements they are associated with. The BA_ section specifies the actual values for the attributes defined in
     * the BA_DEF_ section.
     */
    static const parse_rv parse_ba_(
        std::string_view rng, const nodes_t& nodes, const attr_types_t& ats_, interpreter& ipt)
    {
        bool has_sect = false;
        std::string attr_name, object_type;
        attr_val_t attr_val;

        unsigned bu_ord;
        std::uint32_t message_id;
        std::string object_name;

        const auto av_glob_ = lazy<attr_val_rule>[to(attr_val)];
        const auto av_bu_ = x3::lexeme[x3::string("BU_")[to(object_type)] >> +x3::blank] >> nodes[to(bu_ord)]
            >> lazy<attr_val_rule>[to(attr_val)];
        const auto av_sg_ = x3::lexeme[x3::string("SG_")[to(object_type)] >> +x3::blank] >> x3::uint_[to(message_id)]
            >> name_[to(object_name)] >> lazy<attr_val_rule>[to(attr_val)];
        const auto av_bo_ = x3::lexeme[x3::string("BO_")[to(object_type)] >> +x3::blank] >> x3::uint_[to(message_id)]
            >> lazy<attr_val_rule>[to(attr_val)];
        const auto av_ev_ = x3::lexeme[x3::string("EV_")[to(object_type)] >> +x3::blank] >> name_[to(object_name)]
            >> lazy<attr_val_rule>[to(attr_val)];

        const auto ba_ = x3::with<attr_val_rule>(attr_val_rule {})[x3::omit[x3::lexeme[x3::lit("BA_") >> +x3::blank]]
            >> x3::attr(true)[to(has_sect)] >> &select_parser<attr_val_rule>(ats_) >> quoted_name_[to(attr_name)]
            >> (av_sg_ | av_bo_ | av_bu_ | av_ev_ | av_glob_) >> od(';') >> end_cmd_];

        for (auto iter = rng.begin(); iter != rng.end(); has_sect = false)
        {
            object_type.clear();

            if (!phrase_parse(iter, rng.end(), ba_, skipper_))
            {
                return make_rv(iter, rng.end(), !has_sect);
            }

            // TODO: can break this up into multiple CPOs

            def_ba(ipt, std::move(attr_name), std::move(object_type), std::move(object_name), bu_ord, message_id,
                std::move(attr_val));
        }
        return make_rv(rng.end(), rng.end(), true);
    }

    /**
     * @brief Parses the VAL_ section of the DBC file.
     *
     * @param rng The input range.
     * @param ipt The interpreter.
     * @return The parse result value.
     *
     * @note The VAL_ section in a DBC file defines the value descriptions for signals or environment variables.
     *       It maps specific values to human-readable descriptions, which helps in interpreting the raw data
     *       transmitted over the CAN network. Each entry in the VAL_ section associates a value with a description
     *       for a particular signal or environment variable.
     */
    static const parse_rv parse_val_(std::string_view rng, interpreter& ipt)
    {
        using val_desc = std::pair<unsigned, std::string>;

        bool has_sect = false;
        std::string signal_name, env_var_name;
        std::optional<std::uint32_t> msg_id;
        std::vector<val_desc> val_descs;

        const auto val_ = x3::omit[x3::lexeme[x3::lit("VAL_") >> +x3::blank]] >> x3::attr(true)[to(has_sect)]
            >> (name_[to(env_var_name)] | (x3::uint_[to(msg_id)] >> name_[to(signal_name)]))
            >> (*as<val_desc>(x3::uint_ >> quoted_name_))[to(val_descs)] >> od(';') >> end_cmd_;

        for (auto iter = rng.begin(); iter != rng.end(); has_sect = false)
        {
            msg_id.reset();
            if (!phrase_parse(iter, rng.end(), val_, skipper_))
            {
                return make_rv(iter, rng.end(), !has_sect);
            }

            if (!msg_id.has_value())
            {
                def_val_env(ipt, env_var_name, std::move(val_descs));
            }
            else
            {
                def_val_sg(ipt, msg_id.value(), signal_name, std::move(val_descs));
            }
        }
        return make_rv(rng.end(), rng.end(), true);
    }

    /**
     * @brief Parses the VAL_TABLE_ section of the DBC file.
     *
     * @param rng The input range.
     * @param val_tables The value tables.
     * @param ipt The interpreter.
     * @return The parse result value.
     *
     * @note The VAL_TABLE_ section in a DBC file defines value tables that map specific values to human-readable
     *       descriptions. These tables are used to interpret the raw data values in a more meaningful way.
     *       Each value table consists of a set of value-description pairs, which can be referenced by signals
     *       or environment variables to provide context for the data being transmitted over the CAN network.
     */
    static const parse_rv parse_val_table_(std::string_view rng, nodes_t& val_tables, interpreter& ipt)
    {
        using val_desc = std::pair<unsigned, std::string>;

        bool has_sect = false;
        std::string table_name;
        std::vector<val_desc> val_descs;

        const auto val_table_ = x3::omit[x3::lexeme[x3::lit("VAL_TABLE_") >> +x3::blank]]
            >> x3::attr(true)[to(has_sect)] >> name_[to(table_name)]
            >> (*as<val_desc>(x3::uint_ >> quoted_name_))[to(val_descs)] >> od(';') >> end_cmd_;

        unsigned val_table_ord = 0;
        for (auto iter = rng.begin(); iter != rng.end(); has_sect = false)
        {
            if (!phrase_parse(iter, rng.end(), val_table_, skipper_))
            {
                return make_rv(iter, rng.end(), !has_sect);
            }

            val_tables.add(table_name, val_table_ord++);

            def_val_table(ipt, std::move(table_name), std::move(val_descs));
        }
        return make_rv(rng.end(), rng.end(), true);
    }

    /**
     * @brief Parses the SIG_VALTYPE_ section of the DBC file.
     *
     * @param rng The input range.
     * @param ipt The interpreter.
     * @return The parse result value.
     *
     * @note The SIG_VALTYPE_ section in a DBC file defines the extended value type for signals.
     *       It specifies the message ID, signal name, and the extended value type, which indicates
     *       how the signal's value should be interpreted. This section helps in providing additional
     *       context for the signal's data representation, such as whether it is an integer, float,
     *       or another type.
     */
    static const parse_rv parse_sig_valtype_(std::string_view rng, interpreter& ipt)
    {
        bool has_sect = false;
        unsigned msg_id;
        std::string sig_name;
        unsigned sig_ext_val_type;

        const auto sig_valtype_ = x3::omit[x3::lexeme[x3::lit("SIG_VALTYPE_") >> +x3::blank]]
            >> x3::attr(true)[to(has_sect)] >> x3::uint_[to(msg_id)] >> name_[to(sig_name)] >> od(':')
            >> &x3::char_("0-3") >> x3::uint_[to(sig_ext_val_type)] >> od(';') >> end_cmd_;

        for (auto iter = rng.begin(); iter != rng.end(); has_sect = false)
        {
            if (!phrase_parse(iter, rng.end(), sig_valtype_, skipper_))
            {
                return make_rv(iter, rng.end(), !has_sect);
            }

            def_sig_valtype(ipt, msg_id, std::move(sig_name), sig_ext_val_type);
        }
        return make_rv(rng.end(), rng.end(), true);
    }

    /**
     * @brief Parses the BO_TX_BU_ section of the DBC file.
     *
     * @param rng The input range.
     * @param ipt The interpreter.
     * @return The parse result value.
     *
     * @note The BO_TX_BU_ section in a DBC file defines the transmitters for a message.
     *       It specifies the message ID and a list of nodes (ECUs) that are responsible
     *       for transmitting the message. This section helps in identifying which nodes
     *       are allowed to send a particular message on the CAN network.
     */
    static const parse_rv parse_bo_tx_bu(std::string_view rng, interpreter& ipt)
    {
        bool has_sect = false;
        unsigned msg_id;
        std::vector<std::string> transmitters;

        const auto sig_valtype_ = x3::omit[x3::lexeme[x3::lit("BO_TX_BU_") >> +x3::blank]]
            >> x3::attr(true)[to(has_sect)] >> x3::uint_[to(msg_id)] >> od(':') >> (name_ % ',')[to(transmitters)]
            >> od(';') >> end_cmd_;

        for (auto iter = rng.begin(); iter != rng.end(); has_sect = false)
        {
            if (!phrase_parse(iter, rng.end(), sig_valtype_, skipper_))
            {
                return make_rv(iter, rng.end(), !has_sect);
            }

            def_bo_tx_bu(ipt, msg_id, std::move(transmitters));
        }
        return make_rv(rng.end(), rng.end(), true);
    }

    /**
     * @brief Parses the SG_MUL_VAL_ section of the DBC file.
     *
     * @param rng The input range.
     * @param ipt The interpreter.
     * @return The parse result value.
     *
     * @note The SG_MUL_VAL_ section in a DBC file defines the multiplexed signal values.
     *       It specifies the message ID, the name of the multiplexed signal, the name of the
     *       multiplex switch, and the value ranges for the multiplexed signal. This section
     *       helps in interpreting the values of signals that are multiplexed within a message,
     *       allowing for more efficient use of the CAN network bandwidth.
     */
    static const parse_rv parse_sg_mul_val_(std::string_view rng, interpreter& ipt)
    {
        using value_range = std::pair<unsigned, unsigned>;

        bool has_sect = false;
        unsigned msg_id;
        std::string mux_sig_name, mux_switch_name;
        std::vector<value_range> val_ranges;

        const auto val_range_ = x3::uint_ >> od('-') >> x3::uint_;

        const auto sg_mul_val_ = x3::omit[x3::lexeme[x3::lit("SG_MUL_VAL_") >> +x3::blank]]
            >> x3::attr(true)[to(has_sect)] >> x3::uint_[to(msg_id)] >> name_[to(mux_sig_name)]
            >> name_[to(mux_switch_name)] >> (as<value_range>(val_range_) % ',')[to(val_ranges)] >> od(';') >> end_cmd_;

        for (auto iter = rng.begin(); iter != rng.end(); has_sect = false)
        {
            if (!phrase_parse(iter, rng.end(), sg_mul_val_, skipper_))
            {
                return make_rv(iter, rng.end(), !has_sect);
            }

            def_sg_mul_val(ipt, msg_id, std::move(mux_sig_name), std::move(mux_switch_name), std::move(val_ranges));
        }
        return make_rv(rng.end(), rng.end(), true);
    }

    /**
     * @brief Finds a phrase in a text and returns the line number where it occurs.
     *
     * @param text The text to search in.
     * @param phrase The phrase to search for.
     * @return The line number where the phrase occurs, or -1 if not found.
     */
    static int find_phrase_in_text(std::string_view text, std::string_view phrase)
    {
        int line_number = 1;
        auto it = text.begin();
        while (it != text.end())
        {
            auto line_end = std::find(it, text.end(), '\n');
            std::string_view line(it, line_end - it);
            if (line.find(phrase) != std::string_view::npos)
            {
                return line_number;
            }
            it = (line_end != text.end()) ? line_end + 1 : line_end;
            ++line_number;
        }
        return -1;
    }

    /**
     * @brief Reports a syntax error.
     *
     * @param where The location of the error.
     * @param what The error message.
     * @return false Always returns false.
     */
    static bool syntax_error(std::string_view dbc_src, std::string_view where, std::string_view what = "")
    {
        auto eol = where.find('\n');
        std::string_view line { where.begin(), eol != std::string_view::npos ? where.begin() + eol : where.end() };
        auto line_nr = find_phrase_in_text(dbc_src, line);
        std::fprintf(stderr, "Syntax error: %s at line %d\n => %s\n", what.data(), line_nr, std::string(line).c_str());
        return false;
    }

    bool parse_dbc(std::string_view dbc_src, interpreter ipt)
    {
        auto pv = skip_blines(dbc_src);
        bool expected = true;

        // Parse VERSION section
        if (std::tie(pv, expected) = parse_version(pv, ipt); !expected)
        {
            return syntax_error(dbc_src, pv, "(in VERSION section)");
        }

        // Parse NS_ section
        if (std::tie(pv, expected) = parse_ns_(pv); !expected)
        {
            return syntax_error(dbc_src, pv, "(in NS_ section)");
        }

        // Parse BS_ section
        if (std::tie(pv, expected) = parse_bs_(pv); !expected)
        {
            return syntax_error(dbc_src, pv, "(in BS_ section: expected correct BS_)");
        }

        nodes_t nodes;

        // Parse BU_ section
        if (std::tie(pv, expected) = parse_bu_(pv, nodes, ipt); !expected)
        {
            return syntax_error(dbc_src, pv, "(in BU_ section: expected correct BU_)");
        }

        nodes_t val_tables;

        // Parse VAL_TABLE_ section
        if (std::tie(pv, expected) = parse_val_table_(pv, val_tables, ipt); !expected)
        {
            return syntax_error(dbc_src, pv, "(in VAL_TABLE_ section)");
        }

        // Parse BO_ section
        if (std::tie(pv, expected) = parse_bo_(pv, nodes, ipt); !expected)
        {
            return syntax_error(dbc_src, pv, "(in BO_ section)");
        }

        // Parse BO_TX_BU_ section
        if (std::tie(pv, expected) = parse_bo_tx_bu(pv, ipt); !expected)
        {
            return syntax_error(dbc_src, pv, "(in BO_TX_BU_ section)");
        }

        // Parse EV_ section
        if (std::tie(pv, expected) = parse_ev_(pv, nodes, ipt); !expected)
        {
            return syntax_error(dbc_src, pv, "(in EV_ section)");
        }

        // Parse ENVVAR_DATA_ section
        if (std::tie(pv, expected) = parse_envvar_data_(pv, ipt); !expected)
        {
            return syntax_error(dbc_src, pv, "(in ENVVAR_DATA_ section)");
        }

        // Parse VAL_ section
        if (std::tie(pv, expected) = parse_val_(pv, ipt); !expected)
        {
            return syntax_error(dbc_src, pv, "(in VAL_ section)");
        }

        // Parse SGTYPE_ section
        if (std::tie(pv, expected) = parse_sgtype_(pv, val_tables, ipt); !expected)
        {
            return syntax_error(dbc_src, pv, "(in SGTYPE_ section)");
        }

        // Parse SIG_GROUP_ section
        if (std::tie(pv, expected) = parse_sig_group_(pv, ipt); !expected)
        {
            return syntax_error(dbc_src, pv, "(in SIG_GROUP_ section)");
        }

        // Parse CM_ section
        if (std::tie(pv, expected) = parse_cm_(pv, nodes, ipt); !expected)
        {
            return syntax_error(dbc_src, pv, "(in CM_ section)");
        }

        attr_types_t attr_types;

        // Parse BA_DEF_ section
        if (std::tie(pv, expected) = parse_ba_def_(pv, attr_types, ipt); !expected)
        {
            return syntax_error(dbc_src, pv, "(in BA_DEF_ section)");
        }

        // Parse BA_DEF_DEF_ section
        if (std::tie(pv, expected) = parse_ba_def_def_(pv, attr_types, ipt); !expected)
        {
            return syntax_error(dbc_src, pv, "(in BA_DEF_DEF_ section)");
        }

        // Parse BA_ section
        if (std::tie(pv, expected) = parse_ba_(pv, nodes, attr_types, ipt); !expected)
        {
            return syntax_error(dbc_src, pv, "(in BA_ section)");
        }

        // Parse VAL_ section again
        if (std::tie(pv, expected) = parse_val_(pv, ipt); !expected)
        {
            return syntax_error(dbc_src, pv, "(in VAL_ section)");
        }

        // Parse SIG_VALTYPE_ section
        if (std::tie(pv, expected) = parse_sig_valtype_(pv, ipt); !expected)
        {
            return syntax_error(dbc_src, pv, "(in SIG_VALTYPE_ section)");
        }

        // Parse SG_MUL_VAL_ section
        if (std::tie(pv, expected) = parse_sg_mul_val_(pv, ipt); !expected)
        {
            return syntax_error(dbc_src, pv, "(in SG_MUL_VAL_ section)");
        }

        // Check for any remaining unparsed content
        if (!pv.empty())
        {
            return syntax_error(dbc_src, pv);
        }

        return true;
    }

} // end namespace can
