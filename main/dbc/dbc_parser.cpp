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
        /**
         * @brief A Boost.Spirit parser rule that matches one or more end-of-line (EOL) characters.
         * 
         * This parser rule uses `x3::omit` to ignore the matched EOL characters in the output,
         * effectively skipping over them during parsing. The `+x3::eol` part specifies that
         * one or more EOL characters should be matched.
         */
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

        /**
         * Parses the VERSION section of the DBC file.
         * 
         * This parser expects the following format:
         * - The keyword "VERSION" followed by one or more blank spaces.
         * - A quoted string representing the version.
         * - The end of the command.
         * 
         * The parser omits the "VERSION" keyword and the following blanks,
         * sets a flag indicating that the VERSION section has been parsed,
         * and extracts the quoted version string into the `version` variable.
         * 
         * The `to(has_sect)` and `to(version)` are semantic actions that
         * store the parsed values into the corresponding variables.
         */
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

        /**
         * @brief Parses the "NS_" section of a DBC file.
         *
         * This parser matches the "NS_" keyword followed by one or more blank spaces and a colon.
         * It then sets the `has_sect` attribute to true, indicating that the "NS_" section is present.
         * The parser skips any additional spaces and processes the symbols defined in `ns_syms_`,
         * ignoring any spaces between them. The parsing ends when the `end_cmd_` rule is encountered.
         *
         * The structure of the parser is as follows:
         * - `x3::omit[x3::lexeme[x3::lit("NS_") >> +x3::blank >> ':']]`: Matches "NS_" followed by spaces and a colon, but omits it from the attribute.
         * - `x3::attr(true)[to(has_sect)]`: Sets the `has_sect` attribute to true.
         * - `x3::omit[+x3::space]`: Skips any additional spaces.
         * - `x3::omit[*(ns_syms_ >> x3::omit[+x3::space])]`: Processes the symbols in `ns_syms_`, ignoring spaces between them.
         * - `end_cmd_`: Ends the parsing when this rule is encountered.
         */
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
        /**
         * @brief Parses the BS_ section of a DBC file.
         *
         * This parser matches the following pattern:
         * - The literal "BS_:" followed by optional bit timing parameters.
         * - The bit timing parameters consist of three unsigned integers separated by colons and commas.
         * - The section ends with the end of the command.
         *
         * The parsed components are:
         * - The "BS_:" literal is omitted from the result.
         * - The optional bit timing parameters are also omitted from the result.
         * - The end of the command is matched using the end_cmd_ parser.
         *
         * This parser is used as part of the Boost.Spirit parsing framework to extract
         * information about the bit timing parameters in a DBC file.
         */
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

        /**
         * @brief Parses the "BU_:" keyword followed by a list of node names.
         *
         * This parser omits the "BU_:" keyword and then parses a list of node names,
         * storing the result in the `node_names` attribute. The parsing is terminated
         * by the `end_cmd_` parser.
         *
         * The `x3::omit` directive is used to ignore the "BU_:" keyword in the output.
         * The `x3::lexeme` directive ensures that the "BU_:" keyword is matched as a
         * whole token. The `*name_` parser matches zero or more node names, and the
         * `to(node_names)` semantic action stores the parsed node names.
         */
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

        /**
         * @brief Parses a multiplexer switch value and an optional multiplexer switch character.
         *
         * This parser expects the following format:
         * - A mandatory 'm' character followed by an unsigned integer, which is stored in sg_mux_switch_val.
         * - An optional 'M' character, which is stored in sg_mux_switch.
         *
         * The parser uses Boost.Spirit to define the grammar and parse the input.
         */
        const auto mux_ = -(x3::char_('m') >> x3::uint_[to(sg_mux_switch_val)]) >> -x3::char_('M')[to(sg_mux_switch)];

        /**
         * Parses a signal (SG_) definition in a DBC file.
         *
         * The parsing sequence includes:
         * - "SG_" keyword followed by one or more spaces (omitted from the result)
         * - A boolean attribute set to true
         * - Signal name
         * - Multiplexer indicator
         * - Colon (':')
         * - Start bit (unsigned integer)
         * - Pipe ('|')
         * - Signal size (unsigned integer)
         * - At symbol ('@')
         * - Byte order (character '0' or '1')
         * - Sign (character '+' or '-')
         * - Opening parenthesis ('(')
         * - Factor (double)
         * - Comma (',')
         * - Offset (double)
         * - Closing parenthesis (')')
         * - Opening square bracket ('[')
         * - Minimum value (double)
         * - Pipe ('|')
         * - Maximum value (double)
         * - Closing square bracket (']')
         * - Unit (quoted string)
         * - List of nodes separated by commas
         * - End of command
         *
         * Each parsed element is passed to a corresponding handler function (e.g., to(sg_name), to(sg_start_bit), etc.).
         */
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

        /**
         * @brief Parses the BO_ section of a DBC file.
         *
         * This parser matches the following pattern:
         * - The literal "BO_" followed by one or more blank spaces (omitted from the result).
         * - Sets the `has_sect` attribute to true.
         * - Parses the CAN ID as an unsigned integer and stores it in `can_id`.
         * - Parses the message name and stores it in `msg_name`.
         * - Matches a colon (':') character.
         * - Parses the message size as an unsigned integer and stores it in `msg_size`.
         * - Parses the transmitter node and stores its order in `transmitter_ord`.
         * - Ensures the command ends properly with `end_cmd_`.
         *
         * This parser is intended to be used with the Boost.Spirit `phrase_parse` function.
         */
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

        /**
         * @brief Parses the access type for an EV_ entry in a DBC file.
         *
         * This parser matches the following pattern:
         * - The literal "DUMMY_NODE_VECTOR".
         * - Optionally followed by the literal "800".
         * - Followed by a single character in the range '0' to '3'.
         *
         * The parsed access type is stored in a string.
         */
        const auto access_type_ = x3::lexeme[x3::lit("DUMMY_NODE_VECTOR") >> -x3::lit("800") >> x3::char_("0-3")];

        /**
         * @brief Parses an EV_ (event) entry in a DBC file using Boost.Spirit.
         *
         * This parser is designed to match and extract information from an EV_ entry in a DBC file.
         * The EV_ entry typically contains information about events, including their name, type, 
         * range, unit, initial value, ID, access type, and access nodes.
         *
         * The parser performs the following steps:
         * 1. Matches and omits the "EV_" literal followed by one or more blank spaces.
         * 2. Sets a flag indicating the presence of the section.
         * 3. Extracts the event name.
         * 4. Matches and omits a colon character.
         * 5. Ensures the next character is within the range '0' to '2'.
         * 6. Extracts the event type as an unsigned integer.
         * 7. Matches and omits an opening square bracket.
         * 8. Extracts the minimum value of the event as a double.
         * 9. Matches and omits a pipe character.
         * 10. Extracts the maximum value of the event as a double.
         * 11. Matches and omits a closing square bracket.
         * 12. Extracts the unit of the event as a quoted string.
         * 13. Extracts the initial value of the event as a double.
         * 14. Extracts the event ID as an unsigned integer.
         * 15. Extracts the access type as a string.
         * 16. Extracts the access nodes as a list of nodes separated by commas.
         * 17. Matches and omits a semicolon.
         * 18. Ends the command.
         *
         * The extracted values are stored in the corresponding variables using the `to` function.
         */
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

        /**
         * @brief Parses the ENVVAR_DATA_ section of a DBC file.
         *
         * This parser matches the following pattern:
         * - The literal "ENVVAR_DATA_" followed by one or more blank spaces (omitted from the result).
         * - Sets the `has_sect` attribute to true.
         * - Parses the environment variable name using the `name_` parser and stores it in `ev_name`.
         * - Matches a colon (':') character.
         * - Parses an unsigned integer representing the data size and stores it in `data_size`.
         * - Matches a semicolon (';') to signify the end of the section.
         * - Ensures the command ends properly with `end_cmd_`.
         *
         * This parser is intended to be used with the Boost.Spirit `phrase_parse` function.
         */
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

        /**
         * @brief Parses a signal type definition from a DBC file.
         *
         * This parser extracts various components of a signal type definition, including:
         * - Signal name
         * - Signal size
         * - Byte order (0 or 1)
         * - Sign (+ or -)
         * - Factor
         * - Offset
         * - Minimum value
         * - Maximum value
         * - Unit
         * - Default value
         * - Value table
         *
         * The format expected is:
         * "name : size @ byte_order sign (factor, offset) [min|max] unit default_val, value_table"
         *
         * @param sg_type_name The name of the signal type.
         * @param sg_size The size of the signal.
         * @param sg_byte_order The byte order of the signal (0 or 1).
         * @param sg_sign The sign of the signal (+ or -).
         * @param sg_factor The factor of the signal.
         * @param sg_offset The offset of the signal.
         * @param sg_min The minimum value of the signal.
         * @param sg_max The maximum value of the signal.
         * @param sg_unit The unit of the signal.
         * @param sg_default_val The default value of the signal.
         * @param val_table_ord The value table associated with the signal.
         */
        const auto sg_type_t_ = name_[to(sg_type_name)] >> od(':') >> x3::uint_[to(sg_size)] >> od('@')
            >> as<char>(x3::char_('0') | x3::char_('1'))[to(sg_byte_order)]
            >> as<char>(x3::char_('+') | x3::char_('-'))[to(sg_sign)] >> od('(') >> x3::double_[to(sg_factor)]
            >> od(',') >> x3::double_[to(sg_offset)] >> od(')') >> od('[') >> x3::double_[to(sg_min)] >> od('|')
            >> x3::double_[to(sg_max)] >> od(']') >> quoted_name_[to(sg_unit)] >> x3::double_[to(sg_default_val)]
            >> od(',') >> val_tables[to(val_table_ord)];

        /**
         * @brief Parses a signal type reference using Boost.Spirit.X3.
         *
         * This parser expects the following format:
         * - An unsigned integer representing the message ID.
         * - A name representing the signal name.
         * - A colon character ':'.
         * - A name representing the signal type name.
         *
         * The parsed values are stored in the corresponding variables using the `to` function.
         */
        const auto sg_type_ref_ = x3::uint_[to(msg_id)] >> name_[to(sg_name)] >> od(':') >> name_[to(sg_type_name)];

        /**
         * @brief Parses the SGTYPE_ section of a DBC file.
         *
         * This parser matches the following pattern:
         * - The literal "SGTYPE_" followed by one or more blank spaces.
         * - Sets a flag indicating the presence of this section.
         * - Parses either a signal type definition or a signal type reference.
         * - Matches a semicolon (';') to signify the end of the section.
         * - Ensures the command ends properly with `end_cmd_`.
         *
         * The parsed values are stored in the corresponding attributes:
         * - `has_sect` is set to true.
         * - `sg_type_t_` parses the signal type definition.
         * - `sg_type_ref_` parses the signal type reference.
         */
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

        /**
         * @brief Parses a signal group definition in a DBC file.
         *
         * This parser matches the following pattern:
         * "SIG_GROUP_" followed by one or more spaces, a message ID, a signal group name,
         * the number of repetitions, a colon, a comma-separated list of signal names, and a semicolon.
         *
         * The parsed values are stored in the provided attributes:
         * - `has_sect`: Set to true if the section is present.
         * - `msg_id`: The message ID.
         * - `sig_group_name`: The name of the signal group.
         * - `repetitions`: The number of repetitions.
         * - `sig_names`: A vector of signal names.
         *
         * The parser uses Boost.Spirit X3 for parsing.
         */
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

        /**
         * Parses a comment string enclosed in double quotes, allowing for escaped quotes and newlines.
         * 
         * The comment string is defined as:
         * - Starting with a double quote (")
         * - Followed by zero or more characters that can be:
         *   - An escaped backslash followed by any character (e.g., \\, \")
         *   - Any character except a double quote (")
         *   - A newline character
         * - Ending with a double quote (")
         * 
         * This parser uses Boost.Spirit's x3::lexeme to ensure the entire sequence is treated as a single token.
         */
        const auto comment_
            = as<std::string>(x3::lexeme['"' >> *(('\\' >> x3::char_("\\\"")) | ~x3::char_('"') | x3::eol) >> '"']);

        const auto cm_glob_ = comment_[to(comment_text)];
        /**
         * @brief Parses a "BU_" comment line in a DBC file.
         *
         * This parser matches the "BU_" keyword followed by one or more blank spaces,
         * then captures the nodes and the comment text associated with the "BU_" line.
         *
         * The parsed components are:
         * - object_type: The type of the object being parsed, set to "BU_".
         * - bu_ord: The order of the nodes being parsed.
         * - comment_text: The comment text associated with the "BU_" line.
         *
         * This parser is used as part of the DBC file parsing process to extract
         * information about the nodes and their associated comments.
         */
        const auto cm_bu_ = x3::lexeme[x3::string("BU_")[to(object_type)] >> +x3::blank] >> nodes[to(bu_ord)]
            >> comment_[to(comment_text)];

        /**
         * @brief Parses a specific format of a CAN message signal comment.
         *
         * This parser expects the following format:
         * "SG_ " followed by a message ID, signal name, and comment text.
         *
         * The format is defined as:
         * - "SG_" followed by one or more blank spaces
         * - An unsigned integer representing the message ID
         * - A signal name
         * - A comment text
         *
         * The parsed components are stored in the following variables:
         * - object_type: Stores the type of the object, which is "SG_" in this case.
         * - message_id: Stores the message ID as an unsigned integer.
         * - object_name: Stores the name of the signal.
         * - comment_text: Stores the comment text associated with the signal.
         */
        const auto cm_sg_ = x3::lexeme[x3::string("SG_")[to(object_type)] >> +x3::blank] >> x3::uint_[to(message_id)]
            >> name_[to(object_name)] >> comment_[to(comment_text)];

        /**
         * @brief Parses a comment line for a CAN message in DBC format.
         *
         * This parser matches the following pattern:
         * - "BO_" followed by one or more blank spaces
         * - An unsigned integer representing the message ID
         * - A comment string
         *
         * The parsed components are stored in the following variables:
         * - object_type: Stores the string "BO_"
         * - message_id: Stores the parsed message ID
         * - comment_text: Stores the parsed comment text
         *
         * This parser is typically used as an input for the boost::spirit::qi::phrase_parse function.
         */
        const auto cm_bo_ = x3::lexeme[x3::string("BO_")[to(object_type)] >> +x3::blank] >> x3::uint_[to(message_id)]
            >> comment_[to(comment_text)];

        /**
         * @brief Parses an EV_ object from the input string.
         *
         * This parser matches the following pattern:
         * - The string "EV_" followed by one or more blank spaces.
         * - The name of the object.
         * - The comment associated with the object.
         *
         * The parsed components are stored in the following variables:
         * - object_type: Stores the type of the object, which is "EV_".
         * - object_name: Stores the name of the object.
         * - comment_text: Stores the comment associated with the object.
         *
         * This parser is used as part of the Boost.Spirit parsing framework.
         */
        const auto cm_ev_ = x3::lexeme[x3::string("EV_")[to(object_type)] >> +x3::blank] >> name_[to(object_name)]
            >> comment_[to(comment_text)];

        /**
         * @brief Parses a comment (CM_) section in a DBC file.
         *
         * This parser is designed to handle the "CM_" sections of a DBC file, which are used for comments.
         * It uses Boost.Spirit X3 to define the parsing rules.
         *
         * The parser performs the following steps:
         * 1. Omits the "CM_" literal followed by one or more blank spaces.
         * 2. Sets an attribute to true to indicate the presence of a comment section.
         * 3. Parses one of the possible comment types: signal (cm_sg_), message (cm_bo_), node (cm_bu_), environment variable (cm_ev_), or global (cm_glob_).
         * 4. Ensures the comment section ends with a semicolon (';').
         * 5. Ends the command parsing.
         *
         * This parser is typically used as part of a larger DBC file parsing routine.
         */
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

        /**
         * @brief Parses the type of DBC object from the input string.
         *
         * This parser matches one of the following DBC object type prefixes:
         * - "BU_" for nodes
         * - "BO_" for messages
         * - "SG_" for signals
         * - "EV_" for environment variables
         *
         * The parser uses Boost.Spirit X3 to lexeme match the object type prefix
         * followed by one or more blank spaces, which are omitted from the result.
         *
         * @return A std::string containing the matched object type prefix.
         */
        const auto obj_type_
            = as<std::string>(x3::lexeme[(x3::string("BU_") | x3::string("BO_") | x3::string("SG_") | x3::string("EV_"))
                >> x3::omit[+x3::blank]]);

        /**
         * @brief Parses a string that specifies a data type ("HEX" or "INT") followed by two integer values.
         *
         * This parser expects the following format:
         * - A string "HEX" or "INT" which is converted to a data type and stored in `data_type`.
         * - Two integer values which are stored in `int_min` and `int_max`.
         *
         * The parser uses Boost.Spirit to lexeme the input and apply the transformations.
         *
         * @param x3::lexeme[x3::string("HEX") | x3::string("INT")] Parses the data type string.
         * @param to(data_type) Converts the parsed data type string to a `data_type` and stores it.
         * @param x3::int_[to(int_min)] Parses the first integer and stores it in `int_min`.
         * @param x3::int_[to(int_max)] Parses the second integer and stores it in `int_max`.
         */
        const auto att_hexint_ = as<std::string>(x3::lexeme[x3::string("HEX") | x3::string("INT")])[to(data_type)]
            >> x3::int_[to(int_min)] >> x3::int_[to(int_max)];

        /**
         * @brief Parses a FLOAT attribute definition in a DBC file.
         *
         * This parser matches the "FLOAT" keyword followed by two double values.
         * The parsed data type is stored in `data_type`, and the double values are stored in `dbl_min` and `dbl_max`.
         *
         * @param x3::lexeme[x3::lit("FLOAT")] Parses the "FLOAT" keyword.
         * @param to(data_type) Converts the parsed "FLOAT" keyword to a `data_type` and stores it.
         * @param x3::double_[to(dbl_min)] Parses the first double value and stores it in `dbl_min`.
         * @param x3::double_[to(dbl_max)] Parses the second double value and stores it in `dbl_max`.
         */
        const auto att_float_ = as<std::string>(x3::lexeme[x3::lit("FLOAT")])[to(data_type)] >> x3::double_[to(dbl_min)]
            >> x3::double_[to(dbl_max)];

        /**
         * @brief Parses a string literal "STRING" and converts it to a std::string.
         * 
         * This line uses Boost.Spirit X3 to define a parser that matches the exact 
         * string "STRING" and converts it to a std::string. The result is then 
         * passed to the `to` function which presumably processes or stores the 
         * parsed string in `data_type`.
         * 
         * @note This parser is part of a larger grammar likely used for parsing 
         * DBC (Database CAN) files.
         */
        const auto att_str_ = as<std::string>(x3::lexeme[x3::lit("STRING")])[to(data_type)];

        /**
         * @brief Parses an enumeration attribute definition in a DBC file.
         *
         * This parser matches the "ENUM" keyword followed by a comma-separated list of quoted names.
         * The parsed data type is stored in `data_type`, and the list of enumeration values is stored in `enum_vals`.
         *
         * @param x3::lexeme[x3::lit("ENUM")] Parses the "ENUM" keyword.
         * @param to(data_type) Converts the parsed "ENUM" keyword to a `data_type` and stores it.
         * @param quoted_name_ % ',' Parses a comma-separated list of quoted names and stores them in `enum_vals`.
         */
        const auto att_enum_
            = as<std::string>(x3::lexeme[x3::lit("ENUM")])[to(data_type)] >> -(quoted_name_ % ',')[to(enum_vals)];


        /**
         * @brief Parses the BA_DEF_ attribute definition in a DBC file.
         *
         * This parser handles the BA_DEF_ keyword followed by optional object type,
         * quoted attribute name, and attribute value which can be an integer, float,
         * string, or enumeration. The parser omits the BA_DEF_ keyword and any
         * following whitespace, and sets the has_sect flag to true. It then parses
         * the optional object type, quoted attribute name, and attribute value,
         * followed by a semicolon and the end of the command.
         *
         * The parsed components are:
         * - Optional object type (obj_type_)
         * - Quoted attribute name (quoted_name_)
         * - Attribute value (att_hexint_, att_float_, att_str_, or att_enum_)
         *
         * The parser uses Boost.Spirit X3 for parsing and expects the input to be
         * lexically analyzed.
         *
         * @param rng The input range to be parsed.
         * @param ats_ The attribute types to be populated.
         * @param ipt The interpreter to be used for defining the attribute.
         * @return parse_rv The result of the parsing operation.
         */
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

        /**
         * @brief Parses a specific attribute value definition in a DBC file.
         *
         * This parser is designed to match and process attribute value definitions
         * that start with either "BA_DEF_DEF_REL_" or "BA_DEF_DEF_". It uses Boost.Spirit
         * X3 to define the parsing rules.
         *
         * The parser performs the following steps:
         * 1. Matches the prefix "BA_DEF_DEF_REL_" or "BA_DEF_DEF_".
         * 2. Skips any subsequent blank spaces.
         * 3. Sets a flag to indicate that the section has been found.
         * 4. Parses the attribute value using the `val_` rule.
         * 5. Ensures the definition ends with a semicolon (';').
         * 6. Marks the end of the command using the `end_cmd_` rule.
         *
         * @note This parser uses the `x3::with` directive to associate the `attr_val_rule`
         *       with the parsing context.
         */
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

        // Define parsers for attribute values based on their context (global, node, signal, message, environment variable)
        const auto av_glob_ = lazy<attr_val_rule>[to(attr_val)];
        
        // Parser for node (BU_) attribute values
        const auto av_bu_ = x3::lexeme[x3::string("BU_")[to(object_type)] >> +x3::blank] >> nodes[to(bu_ord)]
            >> lazy<attr_val_rule>[to(attr_val)];
        
        // Parser for signal (SG_) attribute values
        const auto av_sg_ = x3::lexeme[x3::string("SG_")[to(object_type)] >> +x3::blank] >> x3::uint_[to(message_id)]
            >> name_[to(object_name)] >> lazy<attr_val_rule>[to(attr_val)];
        
        // Parser for message (BO_) attribute values
        const auto av_bo_ = x3::lexeme[x3::string("BO_")[to(object_type)] >> +x3::blank] >> x3::uint_[to(message_id)]
            >> lazy<attr_val_rule>[to(attr_val)];
        
        // Parser for environment variable (EV_) attribute values
        const auto av_ev_ = x3::lexeme[x3::string("EV_")[to(object_type)] >> +x3::blank] >> name_[to(object_name)]
            >> lazy<attr_val_rule>[to(attr_val)];

        /**
         * @brief Parses the "BA_" attribute value in a DBC file.
         *
         * This parser is designed to handle the "BA_" attribute value in a DBC (Database Container) file.
         * It uses Boost.Spirit X3 to define the parsing rules and actions.
         *
         * The parsing sequence is as follows:
         * - Matches the literal "BA_" followed by one or more blank spaces (omitted from the attribute value).
         * - Sets a boolean attribute to true indicating the presence of a section.
         * - Selects the appropriate parser for the attribute value rule.
         * - Parses a quoted name and assigns it to the attribute name.
         * - Parses one of the possible attribute value types (signal, message, node, environment variable, or global).
         * - Matches a semicolon to end the command.
         * - Ends the command parsing.
         *
         * @param attr_val_rule The rule for parsing attribute values.
         * @param ats_ The attribute value rule selector.
         * @param quoted_name_ The rule for parsing quoted names.
         * @param av_sg_ The rule for parsing signal attribute values.
         * @param av_bo_ The rule for parsing message attribute values.
         * @param av_bu_ The rule for parsing node attribute values.
         * @param av_ev_ The rule for parsing environment variable attribute values.
         * @param av_glob_ The rule for parsing global attribute values.
         * @param od The rule for matching a semicolon.
         * @param end_cmd_ The rule for ending the command parsing.
         */
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

        /**
         * @brief Parses a VAL_ section in a DBC file.
         *
         * This parser is designed to match and process the VAL_ section of a DBC file.
         * The VAL_ section defines value descriptions for signals or environment variables.
         *
         * The parsing sequence is as follows:
         * - Matches the literal "VAL_" followed by one or more blank spaces, which is omitted from the result.
         * - Sets a boolean attribute to true indicating the presence of the VAL_ section.
         * - Parses either an environment variable name or a message ID followed by a signal name.
         * - Parses zero or more value descriptions, each consisting of an unsigned integer followed by a quoted name.
         * - Matches an optional delimiter (';') and the end of the command.
         *
         * @param to(has_sect) A function to set the presence of the VAL_ section.
         * @param to(env_var_name) A function to set the environment variable name.
         * @param to(msg_id) A function to set the message ID.
         * @param to(signal_name) A function to set the signal name.
         * @param to(val_descs) A function to set the value descriptions.
         * @param od A parser for an optional delimiter.
         * @param end_cmd_ A parser for the end of the command.
         */
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

        /**
         * @brief Parses a VAL_TABLE_ entry in a DBC file.
         *
         * This parser matches the following pattern:
         * - The literal "VAL_TABLE_" followed by one or more blank spaces (omitted from the result).
         * - Sets the `has_sect` attribute to true.
         * - Parses the table name using the `name_` parser and stores it in `table_name`.
         * - Parses zero or more value descriptions, each consisting of an unsigned integer followed by a quoted name,
         *   and stores them in `val_descs`.
         * - Matches a semicolon (';') to signify the end of the value table entry.
         * - Ensures the command ends properly with `end_cmd_`.
         *
         * This parser is intended to be used with the Boost.Spirit `phrase_parse` function.
         */
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

        /**
         * @brief Parses the SIG_VALTYPE_ section of a DBC file.
         *
         * This parser matches the following pattern:
         * - The literal string "SIG_VALTYPE_" followed by one or more blank spaces.
         * - Sets a flag indicating the presence of this section.
         * - An unsigned integer representing the message ID.
         * - A signal name.
         * - A colon character.
         * - A single character in the range '0' to '3'.
         * - An unsigned integer representing the signal extended value type.
         * - A semicolon character.
         * - The end of the command.
         *
         * The parsed values are stored in the corresponding attributes:
         * - `has_sect` is set to true.
         * - `msg_id` stores the message ID.
         * - `sig_name` stores the signal name.
         * - `sig_ext_val_type` stores the signal extended value type.
         */
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

        /**
         * @brief Parses a signal value type from the input.
         *
         * This parser omits the "BO_TX_BU_" literal followed by one or more blank spaces.
         * It then sets a boolean attribute to true and assigns it to `has_sect`.
         * The parser extracts an unsigned integer and assigns it to `msg_id`.
         * It expects a colon (':') followed by a list of names separated by commas,
         * which are assigned to `transmitters`.
         * The parser expects a semicolon (';') to terminate the command.
         * The `end_cmd_` parser is used to signify the end of the command.
         *
         * @param has_sect A boolean attribute indicating the presence of a section.
         * @param msg_id An unsigned integer representing the message ID.
         * @param transmitters A list of names representing the transmitters.
         * @return A parser that matches the described pattern.
         */
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

        /**
         * @brief Parses the "SG_MUL_VAL_" section of a DBC file.
         *
         * This parser is designed to match and extract information from the "SG_MUL_VAL_" section
         * of a DBC (Database Container) file, which is used in CAN (Controller Area Network) communication.
         * 
         * The parser expects the following format:
         * 
         * SG_MUL_VAL_ <msg_id> <mux_sig_name> <mux_switch_name> <val_range_1>,<val_range_2>,...;
         * 
         * - "SG_MUL_VAL_" is a literal string that indicates the start of the section.
         * - <msg_id> is an unsigned integer representing the message ID.
         * - <mux_sig_name> is the name of the multiplexed signal.
         * - <mux_switch_name> is the name of the multiplex switch.
         * - <val_range_1>,<val_range_2>,... is a comma-separated list of value ranges.
         * - The section ends with a semicolon (';').
         * 
         * The parsed information is stored in the following variables:
         * - has_sect: A boolean indicating the presence of the section.
         * - msg_id: The message ID.
         * - mux_sig_name: The name of the multiplexed signal.
         * - mux_switch_name: The name of the multiplex switch.
         * - val_ranges: A list of value ranges.
         * 
         * The parser uses Boost.Spirit.X3 for parsing and expects to be used with the
         * boost::spirit::x3::phrase_parse function.
         */
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
