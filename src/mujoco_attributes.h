#pragma once

#include <godot_cpp/variant/string_name.hpp>
#include <godot_cpp/variant/variant.hpp>

// Class-able attributes: the element properties a MuJoCo default class can set.
//
// An element stores only the attributes set on it, its overrides (MJCF stores only
// the attributes written on an element in the same way). Every other attribute is
// inherited: from the element's default class, or else the base default below.
//
// Tables of these are static data, so they hold plain numbers rather than Variants
// (a static Variant would outlive the engine it belongs to).
struct MujocoAttribute {
    const char *key;            // key in the element's overrides
    godot::Variant::Type type;  // BOOL, INT, FLOAT, VECTOR2, VECTOR3 or COLOR
    double base[4];             // value when nothing sets it, as up to 4 numbers

    godot::Variant get_base() const;
    // Converts p_value to this attribute's type. Returns false if it can't.
    bool convert(const godot::Variant &p_value, godot::Variant &r_value) const;
};

struct MujocoAttributeTable {
    const MujocoAttribute *attributes;
    int count;

    // The attribute with key p_key, or nullptr.
    const MujocoAttribute *find(const godot::StringName &p_key) const;
};

// Defines a table from a static array.
#define MUJOCO_ATTRIBUTE_TABLE(m_array) MujocoAttributeTable{ m_array, static_cast<int>(sizeof(m_array) / sizeof(m_array[0])) }
