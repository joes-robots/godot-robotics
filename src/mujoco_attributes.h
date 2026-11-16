#pragma once

#include <godot_cpp/core/property_info.hpp>
#include <godot_cpp/variant/variant.hpp>

// One attribute of a MuJoCo element, as shown in Godot: for example a geom's
// "density". Each element class has a static table of these.
//
// The table only describes the property (its name, type and inspector hint).
// What the attribute means in MuJoCo, i.e. which field of the spec it reads and
// writes, is in the element class's read and write functions.
struct MujocoAttribute {
    const char *name;
    godot::Variant::Type type;
    godot::PropertyHint hint;
    const char *hint_string;
};
