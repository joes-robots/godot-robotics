#include "mujoco_options.h"

#include <cstddef>

#include <godot_cpp/core/error_macros.hpp>
#include <godot_cpp/variant/string.hpp>

#include "mujoco_convert.h"

using namespace godot;

namespace {

constexpr const char *PREFIX = "option_";

// How an option is stored in mjOption and shown in Godot.
enum class Kind {
    REAL,  // mjtNum                    <-> float
    INT,   // int                       <-> int
    ENUM,  // int, values 0..count-1    <-> int with a drop-down
    FLAGS, // int used as a bitmask     <-> int with checkboxes
    VEC3,  // mjtNum[3], MuJoCo's frame <-> Vector3, Godot's frame
};

// One row of the table: everything needed to expose one mjOption field.
struct Option {
    const char *name; // property name after PREFIX
    Kind kind;
    size_t offset;    // where the field sits inside mjOption, in bytes
    const char *hint; // ENUM and FLAGS: comma-separated labels; otherwise unused
    int count;        // ENUM: number of valid values; otherwise unused
};

// The ENUM labels below are listed in the order of MuJoCo's enum values. If a
// MuJoCo update changes those values, fail the build instead of mislabelling them.
static_assert(mjINT_EULER == 0 && mjINT_RK4 == 1 && mjINT_IMPLICIT == 2 && mjINT_IMPLICITFAST == 3);
static_assert(mjCONE_PYRAMIDAL == 0 && mjCONE_ELLIPTIC == 1);
static_assert(mjJAC_DENSE == 0 && mjJAC_SPARSE == 1 && mjJAC_AUTO == 2);
static_assert(mjSOL_PGS == 0 && mjSOL_CG == 1 && mjSOL_NEWTON == 2);

#define FIELD(member) offsetof(mjOption, member)

// The order here is the order shown in the inspector.
// To expose another mjOption field, add a row.
const Option OPTIONS[] = {
    { "timestep", Kind::REAL, FIELD(timestep), "", 0 },
    { "impratio", Kind::REAL, FIELD(impratio), "", 0 },
    { "gravity", Kind::VEC3, FIELD(gravity), "", 0 },
    { "wind", Kind::VEC3, FIELD(wind), "", 0 },
    { "magnetic", Kind::VEC3, FIELD(magnetic), "", 0 },
    { "density", Kind::REAL, FIELD(density), "", 0 },
    { "viscosity", Kind::REAL, FIELD(viscosity), "", 0 },
    { "o_margin", Kind::REAL, FIELD(o_margin), "", 0 },
    { "integrator", Kind::ENUM, FIELD(integrator), "Euler,RK4,Implicit,Implicit Fast", 4 },
    { "cone", Kind::ENUM, FIELD(cone), "Pyramidal,Elliptic", 2 },
    { "jacobian", Kind::ENUM, FIELD(jacobian), "Dense,Sparse,Auto", 3 },
    { "solver", Kind::ENUM, FIELD(solver), "PGS,CG,Newton", 3 },
    { "iterations", Kind::INT, FIELD(iterations), "", 0 },
    { "tolerance", Kind::REAL, FIELD(tolerance), "", 0 },
    { "ls_iterations", Kind::INT, FIELD(ls_iterations), "", 0 },
    { "ls_tolerance", Kind::REAL, FIELD(ls_tolerance), "", 0 },
    { "noslip_iterations", Kind::INT, FIELD(noslip_iterations), "", 0 },
    { "noslip_tolerance", Kind::REAL, FIELD(noslip_tolerance), "", 0 },
    { "ccd_iterations", Kind::INT, FIELD(ccd_iterations), "", 0 },
    { "ccd_tolerance", Kind::REAL, FIELD(ccd_tolerance), "", 0 },
    { "sleep_tolerance", Kind::REAL, FIELD(sleep_tolerance), "", 0 },
    { "sdf_iterations", Kind::INT, FIELD(sdf_iterations), "", 0 },
    { "sdf_initpoints", Kind::INT, FIELD(sdf_initpoints), "", 0 },
    { "actuatorgroupdisable", Kind::FLAGS, FIELD(disableactuator), "Group 0,Group 1,Group 2,Group 3,Group 4,Group 5,Group 6", 0 },
};

#undef FIELD

// The field of type T that sits p_offset bytes into p_options.
template <typename T>
T *field(mjOption &p_options, size_t p_offset) {
    return reinterpret_cast<T *>(reinterpret_cast<char *>(&p_options) + p_offset);
}

template <typename T>
const T *field(const mjOption &p_options, size_t p_offset) {
    return reinterpret_cast<const T *>(reinterpret_cast<const char *>(&p_options) + p_offset);
}

// The table row for a property name such as "option_timestep", or nullptr.
const Option *find(const StringName &p_name) {
    const String name = p_name;
    if (!name.begins_with(PREFIX)) {
        return nullptr;
    }
    const String short_name = name.substr(String(PREFIX).length());
    for (const Option &option : OPTIONS) {
        if (short_name == option.name) {
            return &option;
        }
    }
    return nullptr;
}

} // namespace

void mujoco_options::get_property_list(List<PropertyInfo> *r_list) {
    // A group entry: properties after it whose names start with PREFIX are shown
    // under an "Options" heading, with the prefix removed from their labels.
    r_list->push_back(PropertyInfo(Variant::NIL, "Options", PROPERTY_HINT_NONE, PREFIX, PROPERTY_USAGE_GROUP));

    for (const Option &option : OPTIONS) {
        const String name = String(PREFIX) + option.name;
        switch (option.kind) {
            case Kind::REAL:
                r_list->push_back(PropertyInfo(Variant::FLOAT, name));
                break;
            case Kind::INT:
                r_list->push_back(PropertyInfo(Variant::INT, name));
                break;
            case Kind::ENUM:
                r_list->push_back(PropertyInfo(Variant::INT, name, PROPERTY_HINT_ENUM, option.hint));
                break;
            case Kind::FLAGS:
                r_list->push_back(PropertyInfo(Variant::INT, name, PROPERTY_HINT_FLAGS, option.hint));
                break;
            case Kind::VEC3:
                r_list->push_back(PropertyInfo(Variant::VECTOR3, name));
                break;
        }
    }
}

bool mujoco_options::get(const mjOption &p_options, const StringName &p_name, Variant &r_value) {
    const Option *option = find(p_name);
    if (!option) {
        return false;
    }
    switch (option->kind) {
        case Kind::REAL:
            r_value = *field<mjtNum>(p_options, option->offset);
            break;
        case Kind::INT:
        case Kind::ENUM:
        case Kind::FLAGS:
            r_value = *field<int>(p_options, option->offset);
            break;
        case Kind::VEC3:
            r_value = mujoco_convert::to_godot(field<mjtNum>(p_options, option->offset));
            break;
    }
    return true;
}

bool mujoco_options::set(mjOption &p_options, const StringName &p_name, const Variant &p_value) {
    const Option *option = find(p_name);
    if (!option) {
        return false;
    }

    // From here on the name is ours, so return true even when the value is
    // rejected: returning false would tell Godot the property doesn't exist.
    const Variant::Type type = p_value.get_type();
    const bool is_number = type == Variant::FLOAT || type == Variant::INT;
    const String name = p_name;

    switch (option->kind) {
        case Kind::REAL:
            ERR_FAIL_COND_V_MSG(!is_number, true, name + String(" must be a number."));
            *field<mjtNum>(p_options, option->offset) = p_value;
            break;
        case Kind::INT:
        case Kind::FLAGS:
            ERR_FAIL_COND_V_MSG(type != Variant::INT, true, name + String(" must be an integer."));
            *field<int>(p_options, option->offset) = p_value;
            break;
        case Kind::ENUM: {
            ERR_FAIL_COND_V_MSG(type != Variant::INT, true, name + String(" must be an integer."));
            const int value = p_value;
            ERR_FAIL_COND_V_MSG(value < 0 || value >= option->count, true,
                    name + String(" must be one of: ") + String(option->hint) + String("."));
            *field<int>(p_options, option->offset) = value;
        } break;
        case Kind::VEC3:
            ERR_FAIL_COND_V_MSG(type != Variant::VECTOR3, true, name + String(" must be a Vector3."));
            mujoco_convert::to_mujoco(p_value, field<mjtNum>(p_options, option->offset));
            break;
    }
    return true;
}

bool mujoco_options::get_default(const StringName &p_name, Variant &r_value) {
    // Defaults come from MuJoCo itself, so they can't drift from the library's.
    mjOption defaults;
    mj_defaultOption(&defaults);
    return get(defaults, p_name, r_value);
}
