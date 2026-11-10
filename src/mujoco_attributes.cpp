#include "mujoco_attributes.h"

#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/variant/vector2i.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/variant/vector3i.hpp>

using namespace godot;

Variant MujocoAttribute::get_base() const
{
    switch (type) {
        case Variant::BOOL:
            return base[0] != 0.0;
        case Variant::INT:
            return static_cast<int64_t>(base[0]);
        case Variant::FLOAT:
            return base[0];
        case Variant::VECTOR2:
            return Vector2(base[0], base[1]);
        case Variant::VECTOR3:
            return Vector3(base[0], base[1], base[2]);
        case Variant::COLOR:
            return Color(base[0], base[1], base[2], base[3]);
        default:
            return Variant();
    }
}

bool MujocoAttribute::convert(const Variant &p_value, Variant &r_value) const
{
    const Variant::Type from = p_value.get_type();
    if (from == type) {
        r_value = p_value;
        return true;
    }
    // Numbers convert between each other (scenes and scripts mix 1 and 1.0).
    const bool from_number = from == Variant::INT || from == Variant::FLOAT || from == Variant::BOOL;
    switch (type) {
        case Variant::BOOL:
            if (from_number) {
                r_value = static_cast<bool>(p_value);
                return true;
            }
            break;
        case Variant::INT:
            if (from_number) {
                r_value = static_cast<int64_t>(p_value);
                return true;
            }
            break;
        case Variant::FLOAT:
            if (from_number) {
                r_value = static_cast<double>(p_value);
                return true;
            }
            break;
        case Variant::VECTOR2:
            if (from == Variant::VECTOR2I) {
                r_value = Vector2(static_cast<Vector2i>(p_value));
                return true;
            }
            break;
        case Variant::VECTOR3:
            if (from == Variant::VECTOR3I) {
                r_value = Vector3(static_cast<Vector3i>(p_value));
                return true;
            }
            break;
        default:
            break;
    }
    return false;
}

const MujocoAttribute *MujocoAttributeTable::find(const StringName &p_key) const
{
    for (int i = 0; i < count; i++) {
        if (p_key == StringName(attributes[i].key)) {
            return &attributes[i];
        }
    }
    return nullptr;
}
