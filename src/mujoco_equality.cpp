#include "mujoco_equality.h"
#include "mujoco_body.h"
#include "mujoco_convert.h"
#include "mujoco_joint.h"
#include "mujoco_site.h"
#include "mujoco_tendon.h"
#include "mujoco_world.h"

#include <godot_cpp/classes/engine.hpp>

using namespace godot;

namespace {

// Solver settings, with MuJoCo's defaults.
const MujocoAttribute EQUALITY_ATTRIBUTES[] = {
    { "solref_timeconst", Variant::FLOAT, { 0.02 } },
    { "solref_dampratio", Variant::FLOAT, { 1.0 } },
    { "solimp_dmin", Variant::FLOAT, { 0.9 } },
    { "solimp_dmax", Variant::FLOAT, { 0.95 } },
    { "solimp_width", Variant::FLOAT, { 0.001 } },
    { "solimp_midpoint", Variant::FLOAT, { 0.5 } },
    { "solimp_power", Variant::FLOAT, { 2.0 } },
};
const MujocoAttributeTable EQUALITY_TABLE = MUJOCO_ATTRIBUTE_TABLE(EQUALITY_ATTRIBUTES);

// p_node's transform relative to its MujocoWorld (which needn't be in the scene tree).
Transform3D to_world(const Node *p_node)
{
    Transform3D result;
    for (const Node *node = p_node; node && !Object::cast_to<MujocoWorld>(node); node = node->get_parent()) {
        if (const Node3D *node3d = Object::cast_to<Node3D>(node)) {
            result = node3d->get_transform() * result;
        }
    }
    return result;
}

} // namespace

const MujocoAttributeTable *MujocoEquality::get_attribute_table() const
{
    return &EQUALITY_TABLE;
}

void MujocoEquality::overrides_changed()
{
    changed();
}

bool MujocoEquality::resolve_object(const NodePath &p_path, const char *p_property, bool p_optional, String &r_name, mjtObj &r_type, String &r_problem) const
{
    const MujocoWorld *world = get_world();
    r_name = String();
    r_type = mjOBJ_UNKNOWN;
    if (p_path.is_empty()) {
        if (p_optional) {
            return true;
        }
        r_problem = vformat("\"%s\" is empty.", p_property);
        return false;
    }
    Node *node = get_node_or_null(p_path);
    const bool bodies = type == TYPE_CONNECT || type == TYPE_WELD;
    if (bodies && node == world) {
        r_name = "world";
        r_type = mjOBJ_BODY;
        return true;
    }
    const MujocoElement *element = Object::cast_to<MujocoElement>(node);
    const char *expected = bodies ? "a MujocoBody, the MujocoWorld or a MujocoSite"
            : type == TYPE_JOINT  ? "a hinge or slide MujocoJoint"
                                  : "a MujocoTendon";
    bool valid = false;
    if (bodies) {
        if (Object::cast_to<MujocoBody>(node)) {
            r_type = mjOBJ_BODY;
            valid = true;
        } else if (Object::cast_to<MujocoSite>(node)) {
            r_type = mjOBJ_SITE;
            valid = true;
        }
    } else if (type == TYPE_JOINT) {
        const MujocoJoint *joint = Object::cast_to<MujocoJoint>(node);
        valid = joint && (joint->get_type() == MujocoJoint::TYPE_HINGE || joint->get_type() == MujocoJoint::TYPE_SLIDE);
        r_type = mjOBJ_JOINT;
    } else {
        const MujocoTendon *tendon = Object::cast_to<MujocoTendon>(node);
        if (tendon) {
            std::vector<MujocoTendonWrap *> wraps;
            bool fixed = false;
            String tendon_problem;
            if (!tendon->get_path(wraps, fixed, tendon_problem)) {
                r_problem = vformat("The tendon \"%s\" points to is left out of the model: %s", p_property, tendon_problem);
                return false;
            }
            valid = true;
        }
        r_type = mjOBJ_TENDON;
    }
    if (!valid) {
        r_problem = vformat("\"%s\" must point to %s.", p_property, expected);
        return false;
    }
    if (element->get_world() != world) {
        r_problem = vformat("\"%s\" points to a node in a different MujocoWorld.", p_property);
        return false;
    }
    // Element names are their node paths relative to the world.
    r_name = String(world->get_path_to(node));
    return true;
}

bool MujocoEquality::resolve(String &r_name1, String &r_name2, mjtObj &r_type, String &r_problem) const
{
    if (!get_world()) {
        r_problem = "Not inside a MujocoWorld.";
        return false;
    }
    mjtObj type1 = mjOBJ_UNKNOWN;
    mjtObj type2 = mjOBJ_UNKNOWN;
    if (!resolve_object(object1, "object1", false, r_name1, type1, r_problem) ||
            !resolve_object(object2, "object2", true, r_name2, type2, r_problem)) {
        return false;
    }
    r_type = type1;
    const bool bodies = type == TYPE_CONNECT || type == TYPE_WELD;
    if (bodies && r_name2.is_empty()) {
        // object2 empty: the world, for bodies; sites need a second site.
        if (type1 == mjOBJ_SITE) {
            r_problem = "Between sites, \"object2\" must be a second MujocoSite.";
            return false;
        }
        r_name2 = "world";
        type2 = mjOBJ_BODY;
    }
    if (bodies && type1 != type2) {
        r_problem = "\"object1\" and \"object2\" must both be bodies (or the world) or both be sites.";
        return false;
    }
    if (r_name1 == r_name2) {
        r_problem = "\"object1\" and \"object2\" are the same.";
        return false;
    }
    return true;
}

mjsBody *MujocoEquality::add_to_spec(mjSpec *p_spec, mjsBody *p_body, const Transform3D & /*p_transform*/, const String &p_name)
{
    String name1, name2, problem;
    mjtObj object_type = mjOBJ_UNKNOWN;
    if (!resolve(name1, name2, object_type, problem)) {
        // Left out, like an actuator without a joint.
        if (!Engine::get_singleton()->is_editor_hint()) {
            WARN_PRINT(vformat("MuJoCo equality '%s' is left out of the model: %s", p_name, problem));
        }
        return p_body;
    }
    mjsEquality *equality = mjs_addEquality(p_spec, get_spec_default(p_spec));
    mjs_setName(equality->element, p_name.utf8().get_data());
    equality->type = static_cast<mjtEq>(type);
    // Like MJCF, only bodies and sites are told apart; joint and tendon
    // constraints leave the object type unset (their type says what they couple).
    equality->objtype = type == TYPE_CONNECT || type == TYPE_WELD ? object_type : mjOBJ_UNKNOWN;
    mjs_setString(equality->name1, name1.utf8().get_data());
    mjs_setString(equality->name2, name2.utf8().get_data());
    equality->active = active;
    write_attributes(equality);

    // Every value is ours: the spec starts with the polynomial's defaults. MuJoCo
    // keeps the torque scale for every type (only welds use it).
    mju_zero(equality->data, mjNEQDATA);
    equality->data[10] = torque_scale;
    switch (type) {
        case TYPE_CONNECT:
            if (object_type == mjOBJ_BODY) {
                // The anchor: this node's position in object1's frame.
                const Node *body1 = get_node_or_null(object1);
                const Vector3 anchor = (to_world(body1).affine_inverse() * to_world(this)).origin;
                mujoco_convert::to_mujoco(anchor, equality->data);
            }
            break;
        case TYPE_WELD:
            // Anchor at body2's origin, and a relative pose of all zeros: MuJoCo uses
            // the bodies' relative pose in the scene (qpos0).
            break;
        case TYPE_JOINT:
        case TYPE_TENDON:
            for (int i = 0; i < 5; i++) {
                equality->data[i] = poly[i];
            }
            break;
    }
    return p_body;
}

void MujocoEquality::write_attributes(mjsEquality *p_equality) const
{
    p_equality->solref[0] = get_solref_timeconst();
    p_equality->solref[1] = get_solref_dampratio();
    p_equality->solimp[0] = get_solimp_dmin();
    p_equality->solimp[1] = get_solimp_dmax();
    p_equality->solimp[2] = get_solimp_width();
    p_equality->solimp[3] = get_solimp_midpoint();
    p_equality->solimp[4] = get_solimp_power();
}

void MujocoEquality::write_class_defaults(mjsDefault *p_default)
{
    write_attributes(p_default->equality);
}

PackedStringArray MujocoEquality::_get_configuration_warnings() const
{
    PackedStringArray warnings = MujocoElement::_get_configuration_warnings();
    String name1, name2, problem;
    mjtObj object_type = mjOBJ_UNKNOWN;
    if (get_world() && !resolve(name1, name2, object_type, problem)) {
        warnings.push_back(problem + " This constraint is left out of the model.");
    }
    return warnings;
}

void MujocoEquality::changed()
{
    update_configuration_warnings();
    request_compile();
}

void MujocoEquality::write_active()
{
    const int id = get_model_id(mjOBJ_EQUALITY);
    if (id < 0) {
        return;
    }
    // The model's initial value too, so that reset() keeps it.
    MujocoWorld *world = get_world();
    world->get_model()->eq_active0[id] = active;
    world->get_data()->eq_active[id] = active;
}

void MujocoEquality::_validate_property(PropertyInfo &p_property) const
{
    const StringName name = p_property.name;
    const bool polynomial = type == TYPE_JOINT || type == TYPE_TENDON;
    if ((name == StringName("torque_scale") && type != TYPE_WELD) ||
            (String(name).begins_with("poly") && !polynomial)) {
        p_property.usage = PROPERTY_USAGE_STORAGE;
    }
    if (name == StringName("object1") || name == StringName("object2")) {
        p_property.hint_string = type == TYPE_JOINT ? "MujocoJoint"
                : type == TYPE_TENDON               ? "MujocoTendon"
                                                    : "MujocoBody,MujocoWorld,MujocoSite";
    }
}

void MujocoEquality::_bind_methods()
{
    BIND_ENUM_CONSTANT(TYPE_CONNECT);
    BIND_ENUM_CONSTANT(TYPE_WELD);
    BIND_ENUM_CONSTANT(TYPE_JOINT);
    BIND_ENUM_CONSTANT(TYPE_TENDON);

    ClassDB::bind_method(D_METHOD("set_type", "type"), &MujocoEquality::set_type);
    ClassDB::bind_method(D_METHOD("get_type"), &MujocoEquality::get_type);
    ClassDB::bind_method(D_METHOD("set_object1", "path"), &MujocoEquality::set_object1);
    ClassDB::bind_method(D_METHOD("get_object1"), &MujocoEquality::get_object1);
    ClassDB::bind_method(D_METHOD("set_object2", "path"), &MujocoEquality::set_object2);
    ClassDB::bind_method(D_METHOD("get_object2"), &MujocoEquality::get_object2);
    ClassDB::bind_method(D_METHOD("set_active", "active"), &MujocoEquality::set_active);
    ClassDB::bind_method(D_METHOD("is_active"), &MujocoEquality::is_active);
#define EQUALITY_BIND_NUMBER(m_name) \
    ClassDB::bind_method(D_METHOD("set_" #m_name, "value"), &MujocoEquality::set_##m_name); \
    ClassDB::bind_method(D_METHOD("get_" #m_name), &MujocoEquality::get_##m_name);
    EQUALITY_BIND_NUMBER(torque_scale)
    EQUALITY_BIND_NUMBER(poly0)
    EQUALITY_BIND_NUMBER(poly1)
    EQUALITY_BIND_NUMBER(poly2)
    EQUALITY_BIND_NUMBER(poly3)
    EQUALITY_BIND_NUMBER(poly4)
    EQUALITY_BIND_NUMBER(solref_timeconst)
    EQUALITY_BIND_NUMBER(solref_dampratio)
    EQUALITY_BIND_NUMBER(solimp_dmin)
    EQUALITY_BIND_NUMBER(solimp_dmax)
    EQUALITY_BIND_NUMBER(solimp_width)
    EQUALITY_BIND_NUMBER(solimp_midpoint)
    EQUALITY_BIND_NUMBER(solimp_power)
#undef EQUALITY_BIND_NUMBER

    ADD_PROPERTY(PropertyInfo(Variant::INT, "type", PROPERTY_HINT_ENUM, "Connect:0,Weld:1,Joint:2,Tendon:3"), "set_type", "get_type");
    ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "object1", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "MujocoBody,MujocoWorld,MujocoSite"), "set_object1", "get_object1");
    ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "object2", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "MujocoBody,MujocoWorld,MujocoSite"), "set_object2", "get_object2");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "active"), "set_active", "is_active");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "torque_scale", PROPERTY_HINT_RANGE, "0,10,0.001,or_greater"), "set_torque_scale", "get_torque_scale");
    ADD_GROUP("Polynomial", "poly");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "poly0", PROPERTY_HINT_RANGE, "-10,10,0.0001,or_less,or_greater"), "set_poly0", "get_poly0");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "poly1", PROPERTY_HINT_RANGE, "-10,10,0.0001,or_less,or_greater"), "set_poly1", "get_poly1");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "poly2", PROPERTY_HINT_RANGE, "-10,10,0.0001,or_less,or_greater"), "set_poly2", "get_poly2");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "poly3", PROPERTY_HINT_RANGE, "-10,10,0.0001,or_less,or_greater"), "set_poly3", "get_poly3");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "poly4", PROPERTY_HINT_RANGE, "-10,10,0.0001,or_less,or_greater"), "set_poly4", "get_poly4");
    // Attribute properties are stored in "overrides", so they are editor-only here.
    ADD_GROUP("Solver", "");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "solref_timeconst", PROPERTY_HINT_RANGE, "0.0001,1,0.0001,or_greater,or_less,suffix:s", PROPERTY_USAGE_EDITOR), "set_solref_timeconst", "get_solref_timeconst");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "solref_dampratio", PROPERTY_HINT_RANGE, "0,2,0.001,or_greater,or_less", PROPERTY_USAGE_EDITOR), "set_solref_dampratio", "get_solref_dampratio");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "solimp_dmin", PROPERTY_HINT_RANGE, "0.0001,0.9999,0.0001", PROPERTY_USAGE_EDITOR), "set_solimp_dmin", "get_solimp_dmin");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "solimp_dmax", PROPERTY_HINT_RANGE, "0.0001,0.9999,0.0001", PROPERTY_USAGE_EDITOR), "set_solimp_dmax", "get_solimp_dmax");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "solimp_width", PROPERTY_HINT_RANGE, "0,1,0.0001,or_greater", PROPERTY_USAGE_EDITOR), "set_solimp_width", "get_solimp_width");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "solimp_midpoint", PROPERTY_HINT_RANGE, "0,1,0.001", PROPERTY_USAGE_EDITOR), "set_solimp_midpoint", "get_solimp_midpoint");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "solimp_power", PROPERTY_HINT_RANGE, "1,10,0.01,or_greater", PROPERTY_USAGE_EDITOR), "set_solimp_power", "get_solimp_power");
}

void MujocoEquality::set_type(Type p_type)
{
    ERR_FAIL_COND_MSG(p_type < TYPE_CONNECT || p_type > TYPE_TENDON, vformat("Unsupported equality type: %d.", p_type));
    type = p_type;
    notify_property_list_changed();
    changed();
}

MujocoEquality::Type MujocoEquality::get_type() const { return type; }

void MujocoEquality::set_object1(const NodePath &p_path)
{
    object1 = p_path;
    changed();
}

NodePath MujocoEquality::get_object1() const { return object1; }

void MujocoEquality::set_object2(const NodePath &p_path)
{
    object2 = p_path;
    changed();
}

NodePath MujocoEquality::get_object2() const { return object2; }

void MujocoEquality::set_active(bool p_active)
{
    // Simulation state, not the model: no recompile.
    active = p_active;
    write_active();
}

bool MujocoEquality::is_active() const { return active; }

void MujocoEquality::set_torque_scale(double p_scale)
{
    ERR_FAIL_COND_MSG(p_scale < 0.0, "Torque scale must not be negative.");
    torque_scale = p_scale;
    changed();
}

double MujocoEquality::get_torque_scale() const { return torque_scale; }

#define EQUALITY_POLY(m_index) \
    void MujocoEquality::set_poly##m_index(double p_value) \
    { \
        poly[m_index] = p_value; \
        changed(); \
    } \
    double MujocoEquality::get_poly##m_index() const { return poly[m_index]; }

EQUALITY_POLY(0)
EQUALITY_POLY(1)
EQUALITY_POLY(2)
EQUALITY_POLY(3)
EQUALITY_POLY(4)

#undef EQUALITY_POLY

// The solver settings are plain numbers, checked by MuJoCo's compiler.
#define EQUALITY_NUMBER_ATTRIBUTE(m_name) \
    void MujocoEquality::set_##m_name(double p_value) \
    { \
        set_attribute(#m_name, p_value); \
        changed(); \
    } \
    double MujocoEquality::get_##m_name() const { return get_attribute(#m_name); }

EQUALITY_NUMBER_ATTRIBUTE(solref_timeconst)
EQUALITY_NUMBER_ATTRIBUTE(solref_dampratio)
EQUALITY_NUMBER_ATTRIBUTE(solimp_dmin)
EQUALITY_NUMBER_ATTRIBUTE(solimp_dmax)
EQUALITY_NUMBER_ATTRIBUTE(solimp_width)
EQUALITY_NUMBER_ATTRIBUTE(solimp_midpoint)
EQUALITY_NUMBER_ATTRIBUTE(solimp_power)

#undef EQUALITY_NUMBER_ATTRIBUTE
