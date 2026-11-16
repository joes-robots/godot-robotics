#pragma once

// MuJoCo's physics options (mjOption) exposed as Godot properties.
//
// The options are described once, in a table in mujoco_options.cpp. These
// functions use that table to list the properties and to read and write them in
// an mjOption. MujocoWorld owns the mjOption and forwards Godot's dynamic
// property callbacks (_get, _set, _get_property_list, ...) here.
//
// Property names are "option_" + MuJoCo's name, e.g. "option_timestep".
// Vector options (gravity, wind, magnetic) are presented in Godot's coordinate
// convention and stored in MuJoCo's.

#include <godot_cpp/core/property_info.hpp>
#include <godot_cpp/templates/list.hpp>
#include <godot_cpp/variant/string_name.hpp>
#include <godot_cpp/variant/variant.hpp>
#include <mujoco/mujoco.h>

namespace mujoco_options {

// Appends the "Options" inspector group and one property per option.
void get_property_list(godot::List<godot::PropertyInfo> *r_list);

// Reads the option called p_name from p_options. Returns false if p_name is not an option.
bool get(const mjOption &p_options, const godot::StringName &p_name, godot::Variant &r_value);

// Writes p_value to the option called p_name in p_options. Returns false if
// p_name is not an option. An invalid value is reported and leaves p_options unchanged.
bool set(mjOption &p_options, const godot::StringName &p_name, const godot::Variant &p_value);

// MuJoCo's own default for the option called p_name. Returns false if p_name is not an option.
bool get_default(const godot::StringName &p_name, godot::Variant &r_value);

} // namespace mujoco_options
