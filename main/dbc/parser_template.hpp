#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "dbc/dbc_parser.hpp"

class your_class
{
    // The parser invokes callbacks that have access to an instance of this class.
    // It can implement custom logic and use only the parts of the .dbc file that are needed.
};

namespace can
{

    inline void tag_invoke(def_version_cpo, your_class& this_, std::string version)
    {
		(void) this_;
		(void) version;
    }

    inline void tag_invoke(def_bu_cpo, your_class& this_, std::vector<std::string> nodes)
    {
		(void) this_;
		(void) nodes;
    }

    inline void tag_invoke(
        def_bo_cpo, your_class& this_, std::uint32_t msg_id, std::string msg_name, std::size_t msg_size, std::size_t transmitter_ord)
    {
		(void) this_;
		(void) msg_id;
		(void) msg_name;
		(void) msg_size;
		(void) transmitter_ord;
    }

    inline void tag_invoke(def_sg_cpo, your_class& this_, std::uint32_t msg_id, std::optional<unsigned> sg_mux_switch_val,
        std::string sg_name, unsigned sg_start_bit, unsigned sg_size, char sg_byte_order, char sg_sign,
        double sg_factor, double sg_offset, double sg_min, double sg_max, std::string sg_unit,
        std::vector<std::size_t> receiver_ords)
    {
		(void) this_;
		(void) msg_id;
		(void) sg_mux_switch_val;
		(void) sg_name;
		(void) sg_start_bit;
		(void) sg_size;
		(void) sg_byte_order;
		(void) sg_sign;
		(void) sg_factor;
		(void) sg_offset;
		(void) sg_min;
		(void) sg_max;
		(void) sg_unit;
		(void) receiver_ords;
    }

    inline void tag_invoke(def_sg_mux_cpo, your_class& this_, std::uint32_t msg_id, std::string sg_name,
        unsigned sg_start_bit, unsigned sg_size, char sg_byte_order, char sg_sign, std::string sg_unit,
        std::vector<std::size_t> receiver_ords)
    {
		(void) this_;
		(void) msg_id;
		(void) sg_name;
		(void) sg_start_bit;
		(void) sg_size;
		(void) sg_byte_order;
		(void) sg_sign;
		(void) sg_unit;
		(void) receiver_ords;
    }

    inline void tag_invoke(def_ev_cpo, your_class& this_, std::string name, unsigned type, double ev_min, double ev_max,
        std::string unit, double initial, unsigned ev_id, std::string access_type,
        std::vector<std::size_t> access_nodes_ords)
    {
		(void) this_;
		(void) name;
		(void) type;
		(void) ev_min;
		(void) ev_max;
		(void) unit;
		(void) initial;
		(void) ev_id;
		(void) access_type;
		(void) access_nodes_ords;
    }

    inline void tag_invoke(def_envvar_data_cpo, your_class& this_, std::string ev_name, unsigned data_size)
    {
		(void) this_;
		(void) ev_name;
		(void) data_size;
    }

    inline void tag_invoke(
        def_sgtype_cpo, your_class& this_, unsigned msg_id, std::string sg_name, std::string sg_type_name)
    {
		(void) this_;
		(void) msg_id;
		(void) sg_name;
		(void) sg_type_name;
    }

    inline void tag_invoke(def_sgtype_ref_cpo, your_class& this_, std::string sg_type_name, unsigned sg_size,
        char sg_byte_order, char sg_sign, double sg_factor, double sg_offset, double sg_min, double sg_max,
        std::string sg_unit, double sg_default_val, std::size_t val_table_ord)
    {
		(void) this_;
		(void) sg_type_name;
		(void) sg_size;
		(void) sg_byte_order;
		(void) sg_sign;
		(void) sg_factor;
		(void) sg_offset;
		(void) sg_min;
		(void) sg_max;
		(void) sg_unit;
		(void) sg_default_val;
		(void) val_table_ord;
    }

    inline void tag_invoke(def_sig_group_cpo, your_class& this_, std::uint32_t msg_id, std::string sig_group_name,
        unsigned repetitions, std::vector<std::string> sig_names)
    {
		(void) this_;
		(void) msg_id;
		(void) sig_group_name;
		(void) repetitions;
		(void) sig_names;		
    }

    inline void tag_invoke(def_cm_glob_cpo, your_class& this_, std::string comment_text)
    {
		(void) this_;
		(void) comment_text;
    }

    inline void tag_invoke(def_cm_bu_cpo, your_class& this_, unsigned bu_ord, std::string comment_text)
    {
		(void) this_;
		(void) bu_ord;
		(void) comment_text;		
    }

    inline void tag_invoke(def_cm_bo_cpo, your_class& this_, std::uint32_t msg_id, std::string comment_text)
    {
		(void) this_;
		(void) msg_id;
		(void) comment_text;		
    }

    inline void tag_invoke(
        def_cm_sg_cpo, your_class& this_, std::uint32_t msg_id, std::string sg_name, std::string comment_text)
    {
		(void) this_;
		(void) msg_id;
		(void) sg_name;
		(void) comment_text;
    }

    inline void tag_invoke(def_cm_ev_cpo, your_class& this_, std::string ev_name, std::string comment_text)
    {
		(void) this_;
		(void) ev_name;
		(void) comment_text;		
    }

    inline void tag_invoke(def_ba_def_enum_cpo, your_class& this_, std::string ba_name, std::string ba_type,
        std::vector<std::string> enum_vals)
    {
		(void) this_;
		(void) ba_name;
		(void) ba_type;
		(void) enum_vals;
    }

    inline void tag_invoke(
        def_ba_def_int_cpo, your_class& this_, std::string ba_name, std::string ba_type, std::int32_t min, std::int32_t max)
    {
		(void) this_;
		(void) ba_name;
		(void) ba_type;
		(void) min;
		(void) max;
    }

    inline void tag_invoke(
        def_ba_def_float_cpo, your_class& this_, std::string ba_name, std::string ba_type, double min, double max)
    {
		(void) this_;
		(void) ba_name;
		(void) ba_type;
		(void) min;
		(void) max;
    }

    inline void tag_invoke(def_ba_def_string_cpo, your_class& this_, std::string ba_name, std::string ba_type)
    {
		(void) this_;
		(void) ba_name;
		(void) ba_type;
    }

    inline void tag_invoke(def_ba_def_def_cpo, your_class& this_, std::string ba_name,
        std::variant<std::int32_t, double, std::string> default_val)
    {
		(void) this_;
		(void) ba_name;
		(void) default_val;
    }

    inline void tag_invoke(def_ba_cpo, your_class& this_, std::string ba_name, std::string object_type,
        std::string object_name, unsigned bu_ord, std::uint32_t msg_id, std::variant<std::int32_t, double, std::string> ba_val)
    {
		(void) this_;
		(void) ba_name;
		(void) object_type;
		(void) object_name;
		(void) bu_ord;
		(void) msg_id;
		(void) ba_val;		
    }

    inline void tag_invoke(def_val_env_cpo, your_class& this_, std::string env_var_name,
        std::vector<std::pair<unsigned, std::string>> val_descs)
    {
		(void) this_;
		(void) env_var_name;
		(void) val_descs;		
    }

    inline void tag_invoke(def_val_sg_cpo, your_class& this_, std::uint32_t msg_id, std::string sg_name,
        std::vector<std::pair<unsigned, std::string>> val_descs)
    {
		(void) this_;
		(void) msg_id;
		(void) sg_name;
		(void) val_descs;		
    }

    inline void tag_invoke(def_val_table_cpo, your_class& this_, std::string table_name,
        std::vector<std::pair<unsigned, std::string>> val_descs)
    {
		(void) this_;
		(void) table_name;
		(void) val_descs;
    }

    inline void tag_invoke(
        def_sig_valtype_cpo, your_class& this_, unsigned msg_id, std::string sg_name, unsigned sg_ext_val_type)
    {
		(void) this_;
		(void) msg_id;
		(void) sg_name;
		(void) sg_ext_val_type;
    }

    inline void tag_invoke(def_bo_tx_bu_cpo, your_class& this_, unsigned msg_id, std::vector<std::string> transmitters)
    {
		(void) this_;
		(void) msg_id;
		(void) transmitters;
    }

    inline void tag_invoke(def_sg_mul_val_cpo, your_class& this_, unsigned msg_id, std::string mux_sg_name,
        std::string mux_switch_name, std::vector<std::pair<unsigned, unsigned>> val_ranges)
    {
		(void) this_;
		(void) msg_id;
		(void) mux_sg_name;
		(void) mux_switch_name;
		(void) val_ranges;
    }

}; // end namespace can
