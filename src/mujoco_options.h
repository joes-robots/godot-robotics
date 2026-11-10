#pragma once

#include <godot_cpp/core/property_info.hpp>
#include <godot_cpp/templates/list.hpp>
#include <godot_cpp/variant/string_name.hpp>
#include <godot_cpp/variant/variant.hpp>
#include <mujoco/mujoco.h>

// Exposes the fields of an mjOption as Godot properties named "option_<field>",
// driven by a single table in mujoco_options.cpp. A node forwards its
// _get/_set/_get_property_list/_property_get_revert hooks to these functions.
namespace mujoco_options {

// Appends the "Options" group and all option properties.
void get_property_list(godot::List<godot::PropertyInfo> *p_list);

// Returns false if p_name is not an option property.
bool get(const mjOption &p_option, const godot::StringName &p_name, godot::Variant &r_value);

// Returns false if p_name is not an option property. An invalid value is
// rejected with an error and leaves p_option untouched; r_accepted reports
// whether the value was valid and written to p_option.
bool set(mjOption &p_option, const godot::StringName &p_name, const godot::Variant &p_value, bool &r_accepted);

// MuJoCo's default for the property (from mj_defaultOption). Returns false if
// p_name is not an option property.
bool get_default(const godot::StringName &p_name, godot::Variant &r_value);

// Binds the enum and flag constants (INTEGRATOR_RK4, DISABLE_CONTACT, ...) to p_class.
void bind_constants(const godot::StringName &p_class);

} // namespace mujoco_options
