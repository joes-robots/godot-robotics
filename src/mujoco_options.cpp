#include "mujoco_options.h"
#include "mujoco_convert.h"

#include <cstddef>
#include <cstdint>
#include <cstring>

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/error_macros.hpp>

using namespace godot;

namespace {

constexpr const char *PREFIX = "option_";

enum class Kind {
    SUBGROUP, // inspector subgroup header; name is the label, hint the property prefix
    REAL,     // mjtNum
    INT,      // int
    ENUM,     // int, values 0..limit-1
    FLAGS,    // int bitmask, only bits in limit allowed
    VEC3,     // mjtNum[3], converted between Godot (Y-up) and MuJoCo (Z-up) frames
};

struct Field {
    const char *name;   // property name after PREFIX (label for SUBGROUP)
    Kind kind;
    size_t offset;      // byte offset into mjOption
    const char *hint;   // range/enum hint string; nullptr for FLAGS (built at runtime)
    int64_t limit = 0;  // ENUM: number of values; FLAGS: mask of valid bits
};

// The enum hint strings below hard-code MuJoCo's values; fail the build if they change.
static_assert(mjINT_EULER == 0 && mjINT_RK4 == 1 && mjINT_IMPLICIT == 2 && mjINT_IMPLICITFAST == 3);
static_assert(mjCONE_PYRAMIDAL == 0 && mjCONE_ELLIPTIC == 1);
static_assert(mjJAC_DENSE == 0 && mjJAC_SPARSE == 1 && mjJAC_AUTO == 2);
static_assert(mjSOL_PGS == 0 && mjSOL_CG == 1 && mjSOL_NEWTON == 2);
static_assert(mjNREF == 2 && mjNIMP == 5, "solref/solimp rows below assume these sizes");

// MuJoCo's actuatorgroupdisable accepts groups 0-30.
constexpr int ACTUATOR_GROUPS = 31;

constexpr int64_t DISABLE_MASK = (int64_t(1) << mjNDISABLE) - 1;
constexpr int64_t ENABLE_MASK = (int64_t(1) << mjNENABLE) - 1;
constexpr int64_t ACTUATOR_MASK = (int64_t(1) << ACTUATOR_GROUPS) - 1;

// Range hints only constrain the inspector; MuJoCo's compiler validates values set from code.
constexpr const char *TOLERANCE = "0,0.01,0.0000000001,or_greater";
constexpr const char *ITERATIONS = "0,1000,1,or_greater";
constexpr const char *FRICTION = "0,2,0.0001,or_greater";

#define OFF(field) offsetof(mjOption, field)
#define OFF_AT(field, i) (offsetof(mjOption, field) + (i) * sizeof(mjtNum))

// Order here is the order shown in the inspector. Properties before the first
// SUBGROUP appear directly under "Options".
const Field FIELDS[] = {
    { "timestep", Kind::REAL, OFF(timestep), "0.00001,0.1,0.00001,or_greater,exp,suffix:s" },
    { "integrator", Kind::ENUM, OFF(integrator), "Euler:0,RK4:1,Implicit:2,Implicit Fast:3", 4 },
    { "gravity", Kind::VEC3, OFF(gravity), "suffix:m/s^2" },
    { "wind", Kind::VEC3, OFF(wind), "suffix:m/s" },
    { "magnetic", Kind::VEC3, OFF(magnetic), "" },
    { "density", Kind::REAL, OFF(density), "0,10000,0.001,or_greater,suffix:kg/m^3" },
    { "viscosity", Kind::REAL, OFF(viscosity), "0,10,0.00001,or_greater" },

    { "Flags", Kind::SUBGROUP, 0, "option_" },
    { "disableflags", Kind::FLAGS, OFF(disableflags), nullptr, DISABLE_MASK },
    { "enableflags", Kind::FLAGS, OFF(enableflags), nullptr, ENABLE_MASK },
    { "actuatorgroupdisable", Kind::FLAGS, OFF(disableactuator), nullptr, ACTUATOR_MASK },

    { "Solver", Kind::SUBGROUP, 0, "option_" },
    { "solver", Kind::ENUM, OFF(solver), "PGS:0,CG:1,Newton:2", 3 },
    { "cone", Kind::ENUM, OFF(cone), "Pyramidal:0,Elliptic:1", 2 },
    { "jacobian", Kind::ENUM, OFF(jacobian), "Dense:0,Sparse:1,Auto:2", 3 },
    { "impratio", Kind::REAL, OFF(impratio), "0.0001,100,0.0001,or_greater,exp" },
    { "iterations", Kind::INT, OFF(iterations), ITERATIONS },
    { "tolerance", Kind::REAL, OFF(tolerance), TOLERANCE },
    { "ls_iterations", Kind::INT, OFF(ls_iterations), ITERATIONS },
    { "ls_tolerance", Kind::REAL, OFF(ls_tolerance), TOLERANCE },
    { "noslip_iterations", Kind::INT, OFF(noslip_iterations), ITERATIONS },
    { "noslip_tolerance", Kind::REAL, OFF(noslip_tolerance), TOLERANCE },
    { "sleep_tolerance", Kind::REAL, OFF(sleep_tolerance), TOLERANCE },

    { "Collision", Kind::SUBGROUP, 0, "option_" },
    { "ccd_iterations", Kind::INT, OFF(ccd_iterations), ITERATIONS },
    { "ccd_tolerance", Kind::REAL, OFF(ccd_tolerance), TOLERANCE },
    { "sdf_iterations", Kind::INT, OFF(sdf_iterations), "1,100,1,or_greater" },
    { "sdf_initpoints", Kind::INT, OFF(sdf_initpoints), "1,100,1,or_greater" },

    // Only used when the Override enable flag is set.
    { "Contact Override", Kind::SUBGROUP, 0, "option_o_" },
    { "o_margin", Kind::REAL, OFF(o_margin), "0,1,0.0001,or_greater,suffix:m" },
    { "o_solref_timeconst", Kind::REAL, OFF_AT(o_solref, 0), "0.0001,1,0.0001,or_greater,or_less,suffix:s" },
    { "o_solref_dampratio", Kind::REAL, OFF_AT(o_solref, 1), "0,2,0.001,or_greater,or_less" },
    { "o_solimp_dmin", Kind::REAL, OFF_AT(o_solimp, 0), "0.0001,0.9999,0.0001" },
    { "o_solimp_dmax", Kind::REAL, OFF_AT(o_solimp, 1), "0.0001,0.9999,0.0001" },
    { "o_solimp_width", Kind::REAL, OFF_AT(o_solimp, 2), "0,1,0.0001,or_greater,suffix:m" },
    { "o_solimp_midpoint", Kind::REAL, OFF_AT(o_solimp, 3), "0,1,0.001" },
    { "o_solimp_power", Kind::REAL, OFF_AT(o_solimp, 4), "1,10,0.01,or_greater" },
    { "o_friction_slide1", Kind::REAL, OFF_AT(o_friction, 0), FRICTION },
    { "o_friction_slide2", Kind::REAL, OFF_AT(o_friction, 1), FRICTION },
    { "o_friction_spin", Kind::REAL, OFF_AT(o_friction, 2), FRICTION },
    { "o_friction_roll1", Kind::REAL, OFF_AT(o_friction, 3), FRICTION },
    { "o_friction_roll2", Kind::REAL, OFF_AT(o_friction, 4), FRICTION },
};

#undef OFF
#undef OFF_AT

template <typename T>
T &field_ref(mjOption &p_option, size_t p_offset) {
    return *reinterpret_cast<T *>(reinterpret_cast<char *>(&p_option) + p_offset);
}

template <typename T>
const T &field_ref(const mjOption &p_option, size_t p_offset) {
    return *reinterpret_cast<const T *>(reinterpret_cast<const char *>(&p_option) + p_offset);
}

const Field *find(const StringName &p_name) {
    const String name = p_name;
    if (!name.begins_with(PREFIX)) {
        return nullptr;
    }
    const String field = name.substr(std::strlen(PREFIX));
    for (const Field &f : FIELDS) {
        if (f.kind != Kind::SUBGROUP && field == f.name) {
            return &f;
        }
    }
    return nullptr;
}

String flags_hint(const char *const *p_labels, int p_count) {
    String hint;
    for (int i = 0; i < p_count; i++) {
        if (i > 0) {
            hint += ",";
        }
        hint += vformat("%s:%d", p_labels[i], int64_t(1) << i);
    }
    return hint;
}

String actuator_groups_hint() {
    String hint;
    for (int i = 0; i < ACTUATOR_GROUPS; i++) {
        if (i > 0) {
            hint += ",";
        }
        hint += vformat("Group %d:%d", i, int64_t(1) << i);
    }
    return hint;
}

String hint_string(const Field &p_field) {
    if (p_field.kind != Kind::FLAGS) {
        return p_field.hint;
    }
    if (p_field.offset == offsetof(mjOption, disableflags)) {
        return flags_hint(mjDISABLESTRING, mjNDISABLE);
    }
    if (p_field.offset == offsetof(mjOption, enableflags)) {
        return flags_hint(mjENABLESTRING, mjNENABLE);
    }
    return actuator_groups_hint();
}

Variant read(const mjOption &p_option, const Field &p_field) {
    switch (p_field.kind) {
        case Kind::REAL:
            return field_ref<mjtNum>(p_option, p_field.offset);
        case Kind::INT:
        case Kind::ENUM:
        case Kind::FLAGS:
            return field_ref<int>(p_option, p_field.offset);
        case Kind::VEC3:
            return mujoco_convert::to_godot(&field_ref<mjtNum>(p_option, p_field.offset));
        case Kind::SUBGROUP:
            break;
    }
    return Variant();
}

bool is_number(const Variant &p_value) {
    return p_value.get_type() == Variant::INT || p_value.get_type() == Variant::FLOAT;
}

// Validates p_value for p_field and writes it. Returns false (after printing an
// error) if the value is rejected.
bool write(mjOption &p_option, const Field &p_field, const Variant &p_value) {
    const String name = String(PREFIX) + p_field.name;

    if (p_field.kind == Kind::REAL) {
        ERR_FAIL_COND_V_MSG(!is_number(p_value), false, vformat("%s expects a number.", name));
        field_ref<mjtNum>(p_option, p_field.offset) = double(p_value);
        return true;
    }

    if (p_field.kind == Kind::VEC3) {
        ERR_FAIL_COND_V_MSG(p_value.get_type() != Variant::VECTOR3, false, vformat("%s expects a Vector3.", name));
        mujoco_convert::to_mujoco(Vector3(p_value), &field_ref<mjtNum>(p_option, p_field.offset));
        return true;
    }

    // Integer kinds
    ERR_FAIL_COND_V_MSG(p_value.get_type() != Variant::INT, false, vformat("%s expects an int.", name));
    const int64_t value = int64_t(p_value);
    switch (p_field.kind) {
        case Kind::INT:
            ERR_FAIL_COND_V_MSG(value < INT32_MIN || value > INT32_MAX, false, vformat("%s is out of range: %d.", name, value));
            break;
        case Kind::ENUM:
            ERR_FAIL_COND_V_MSG(value < 0 || value >= p_field.limit, false, vformat("%s has no value %d.", name, value));
            break;
        case Kind::FLAGS:
            ERR_FAIL_COND_V_MSG(value < 0 || (value & ~p_field.limit) != 0, false, vformat("%s has unknown bits set: %d.", name, value));
            break;
        default:
            return false;
    }
    field_ref<int>(p_option, p_field.offset) = static_cast<int>(value);
    return true;
}

const mjOption &default_option() {
    static const mjOption defaults = [] {
        mjOption option;
        mj_defaultOption(&option);
        return option;
    }();
    return defaults;
}

} // namespace

namespace mujoco_options {

void get_property_list(List<PropertyInfo> *p_list) {
    p_list->push_back(PropertyInfo(Variant::NIL, "Options", PROPERTY_HINT_NONE, PREFIX, PROPERTY_USAGE_GROUP));

    for (const Field &f : FIELDS) {
        const String name = String(PREFIX) + f.name;
        switch (f.kind) {
            case Kind::SUBGROUP:
                p_list->push_back(PropertyInfo(Variant::NIL, f.name, PROPERTY_HINT_NONE, f.hint, PROPERTY_USAGE_SUBGROUP));
                break;
            case Kind::REAL:
                p_list->push_back(PropertyInfo(Variant::FLOAT, name, PROPERTY_HINT_RANGE, f.hint));
                break;
            case Kind::INT:
                p_list->push_back(PropertyInfo(Variant::INT, name, PROPERTY_HINT_RANGE, f.hint));
                break;
            case Kind::ENUM:
                p_list->push_back(PropertyInfo(Variant::INT, name, PROPERTY_HINT_ENUM, f.hint));
                break;
            case Kind::FLAGS:
                p_list->push_back(PropertyInfo(Variant::INT, name, PROPERTY_HINT_FLAGS, hint_string(f)));
                break;
            case Kind::VEC3:
                p_list->push_back(PropertyInfo(Variant::VECTOR3, name, PROPERTY_HINT_NONE, f.hint));
                break;
        }
    }
}

bool get(const mjOption &p_option, const StringName &p_name, Variant &r_value) {
    const Field *field = find(p_name);
    if (!field) {
        return false;
    }
    r_value = read(p_option, *field);
    return true;
}

bool set(mjOption &p_option, const StringName &p_name, const Variant &p_value, bool &r_changed) {
    r_changed = false;
    const Field *field = find(p_name);
    if (!field) {
        return false;
    }
    r_changed = write(p_option, *field, p_value);
    return true;
}

bool get_default(const StringName &p_name, Variant &r_value) {
    return get(default_option(), p_name, r_value);
}

void bind_constants(const StringName &p_class) {
    ClassDB::bind_integer_constant(p_class, "Integrator", "INTEGRATOR_EULER", mjINT_EULER);
    ClassDB::bind_integer_constant(p_class, "Integrator", "INTEGRATOR_RK4", mjINT_RK4);
    ClassDB::bind_integer_constant(p_class, "Integrator", "INTEGRATOR_IMPLICIT", mjINT_IMPLICIT);
    ClassDB::bind_integer_constant(p_class, "Integrator", "INTEGRATOR_IMPLICIT_FAST", mjINT_IMPLICITFAST);

    ClassDB::bind_integer_constant(p_class, "Cone", "CONE_PYRAMIDAL", mjCONE_PYRAMIDAL);
    ClassDB::bind_integer_constant(p_class, "Cone", "CONE_ELLIPTIC", mjCONE_ELLIPTIC);

    ClassDB::bind_integer_constant(p_class, "Jacobian", "JACOBIAN_DENSE", mjJAC_DENSE);
    ClassDB::bind_integer_constant(p_class, "Jacobian", "JACOBIAN_SPARSE", mjJAC_SPARSE);
    ClassDB::bind_integer_constant(p_class, "Jacobian", "JACOBIAN_AUTO", mjJAC_AUTO);

    ClassDB::bind_integer_constant(p_class, "Solver", "SOLVER_PGS", mjSOL_PGS);
    ClassDB::bind_integer_constant(p_class, "Solver", "SOLVER_CG", mjSOL_CG);
    ClassDB::bind_integer_constant(p_class, "Solver", "SOLVER_NEWTON", mjSOL_NEWTON);

    // Names come from MuJoCo, e.g. "NativeCCD" -> DISABLE_NATIVE_CCD.
    for (int i = 0; i < mjNDISABLE; i++) {
        const String constant = "DISABLE_" + String(mjDISABLESTRING[i]).to_snake_case().to_upper();
        ClassDB::bind_integer_constant(p_class, "DisableFlags", constant, int64_t(1) << i, true);
    }
    for (int i = 0; i < mjNENABLE; i++) {
        const String constant = "ENABLE_" + String(mjENABLESTRING[i]).to_snake_case().to_upper();
        ClassDB::bind_integer_constant(p_class, "EnableFlags", constant, int64_t(1) << i, true);
    }
}

} // namespace mujoco_options
