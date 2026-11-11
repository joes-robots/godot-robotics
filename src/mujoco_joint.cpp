#include "mujoco_joint.h"
#include "mujoco_body.h"
#include "mujoco_convert.h"
#include "mujoco_world.h"

#include <godot_cpp/classes/engine.hpp>

using namespace godot;

namespace {

const MujocoAttribute JOINT_ATTRIBUTES[] = {
    { "type", Variant::INT, { MujocoJoint::TYPE_HINGE } },
    { "limited", Variant::BOOL, { 0 } },
    { "range", Variant::VECTOR2, { -Math::PI / 2, Math::PI / 2 } },
    { "damping", Variant::FLOAT, { 0 } },      // force (or torque) per unit velocity
    { "stiffness", Variant::FLOAT, { 0 } },    // spring force (or torque) per unit displacement
    { "armature", Variant::FLOAT, { 0 } },     // extra rotor inertia (or mass for slide)
    { "frictionloss", Variant::FLOAT, { 0 } }, // dry friction force (or torque)
    { "ref", Variant::FLOAT, { 0 } },          // joint position of the scene pose
    { "springref", Variant::FLOAT, { 0 } },    // spring rest position
    // Limit behavior, with MuJoCo's defaults.
    { "limit_margin", Variant::FLOAT, { 0 } },
    { "limit_solref_timeconst", Variant::FLOAT, { 0.02 } },
    { "limit_solref_dampratio", Variant::FLOAT, { 1.0 } },
    { "limit_solimp_dmin", Variant::FLOAT, { 0.9 } },
    { "limit_solimp_dmax", Variant::FLOAT, { 0.95 } },
    { "limit_solimp_width", Variant::FLOAT, { 0.001 } },
    { "limit_solimp_midpoint", Variant::FLOAT, { 0.5 } },
    { "limit_solimp_power", Variant::FLOAT, { 2.0 } },
};
const MujocoAttributeTable JOINT_TABLE = MUJOCO_ATTRIBUTE_TABLE(JOINT_ATTRIBUTES);

} // namespace

const MujocoAttributeTable *MujocoJoint::get_attribute_table() const
{
    return &JOINT_TABLE;
}

void MujocoJoint::overrides_changed()
{
    notify_property_list_changed();
    changed();
}

mjsBody *MujocoJoint::add_to_spec(mjSpec *p_spec, mjsBody *p_body, const Transform3D &p_transform, const String &p_name)
{
    if (get_type() == TYPE_FREE) {
        // A free joint has no anchor, axis, limits or spring.
        mjsJoint *joint = mjs_addFreeJoint(p_body);
        mjs_setName(joint->element, p_name.utf8().get_data());
        mjs_setDefault(joint->element, get_spec_default(p_spec));
        return p_body;
    }

    mjsJoint *joint = mjs_addJoint(p_body, get_spec_default(p_spec));
    mjs_setName(joint->element, p_name.utf8().get_data());
    write_attributes(joint);

    mjtNum quat_unused[4];
    mujoco_convert::to_mujoco(p_transform, joint->pos, quat_unused);
    // The node's local Y axis, expressed in the body's frame.
    mujoco_convert::to_mujoco(p_transform.basis.xform(Vector3(0, 1, 0)).normalized(), joint->axis);
    return p_body;
}

void MujocoJoint::write_attributes(mjsJoint *p_joint) const
{
    const Type type = get_type();
    p_joint->type = static_cast<mjtJoint>(type);
    // The world's compiler takes radians, matching the range units.
    const Vector2 range = get_range();
    p_joint->limited = is_limited() ? mjLIMITED_TRUE : mjLIMITED_FALSE;
    p_joint->range[0] = type == TYPE_BALL ? 0.0 : range.x;
    p_joint->range[1] = range.y;
    p_joint->damping[0] = get_damping();
    p_joint->stiffness[0] = get_stiffness();
    p_joint->armature = get_armature();
    p_joint->frictionloss = get_frictionloss();
    p_joint->ref = get_ref();
    p_joint->springref = get_springref();
    p_joint->margin = get_limit_margin();
    p_joint->solref_limit[0] = get_limit_solref_timeconst();
    p_joint->solref_limit[1] = get_limit_solref_dampratio();
    p_joint->solimp_limit[0] = get_limit_solimp_dmin();
    p_joint->solimp_limit[1] = get_limit_solimp_dmax();
    p_joint->solimp_limit[2] = get_limit_solimp_width();
    p_joint->solimp_limit[3] = get_limit_solimp_midpoint();
    p_joint->solimp_limit[4] = get_limit_solimp_power();
}

void MujocoJoint::write_class_defaults(mjsDefault *p_default)
{
    write_attributes(p_default->joint);
}

void MujocoJoint::inherited_changed()
{
    MujocoElement::inherited_changed();
    update_gizmos();
}

PackedStringArray MujocoJoint::_get_configuration_warnings() const
{
    PackedStringArray warnings = MujocoElement::_get_configuration_warnings();

    const MujocoWorld *world = get_world();
    bool in_body = false;
    for (const Node *node = get_parent(); node && node != world; node = node->get_parent()) {
        if (Object::cast_to<MujocoBody>(node)) {
            in_body = true;
            break;
        }
    }
    if (world && !in_body) {
        warnings.push_back("Must be inside a MujocoBody; the world body cannot have joints.");
    }
    const Type type = get_type();
    if (is_limited() && type != TYPE_FREE && type != TYPE_BALL && get_range().x >= get_range().y) {
        warnings.push_back("Range minimum must be less than its maximum.");
    }
    return warnings;
}

void MujocoJoint::changed()
{
    update_configuration_warnings();
    update_gizmos();
    request_compile();
}

bool MujocoJoint::get_state_index(int &r_qpos, int &r_dof)
{
    const Type type = get_type();
    ERR_FAIL_COND_V_MSG(type == TYPE_FREE || type == TYPE_BALL, false,
            "Only hinge and slide joints have a single position and velocity.");
    const int id = get_model_id(mjOBJ_JOINT);
    ERR_FAIL_COND_V_MSG(id < 0, false, "This joint is not part of a compiled MuJoCo model.");
    const mjModel *model = get_world()->get_model();
    r_qpos = model->jnt_qposadr[id];
    r_dof = model->jnt_dofadr[id];
    return true;
}

double MujocoJoint::get_joint_position()
{
    int qpos = 0, dof = 0;
    if (!get_state_index(qpos, dof)) {
        return 0.0;
    }
    return get_world()->get_data()->qpos[qpos];
}

void MujocoJoint::set_joint_position(double p_position)
{
    ERR_FAIL_COND_MSG(Engine::get_singleton()->is_editor_hint(), "Joint positions can only be set while the game runs.");
    int qpos = 0, dof = 0;
    if (!get_state_index(qpos, dof)) {
        return;
    }
    get_world()->get_data()->qpos[qpos] = p_position;
    get_world()->state_changed();
}

double MujocoJoint::get_joint_velocity()
{
    int qpos = 0, dof = 0;
    if (!get_state_index(qpos, dof)) {
        return 0.0;
    }
    return get_world()->get_data()->qvel[dof];
}

void MujocoJoint::set_joint_velocity(double p_velocity)
{
    ERR_FAIL_COND_MSG(Engine::get_singleton()->is_editor_hint(), "Joint velocities can only be set while the game runs.");
    int qpos = 0, dof = 0;
    if (!get_state_index(qpos, dof)) {
        return;
    }
    get_world()->get_data()->qvel[dof] = p_velocity;
    get_world()->state_changed();
}

void MujocoJoint::_validate_property(PropertyInfo &p_property) const
{
    // Free joints have no axis, so no limits or spring.
    const Type type = get_type();
    const bool has_axis = type != TYPE_FREE;
    const StringName name = p_property.name;

    const bool single_value = type == TYPE_HINGE || type == TYPE_SLIDE;
    if ((name == StringName("ref") || name == StringName("springref"))) {
        if (!single_value) {
            p_property.usage = PROPERTY_USAGE_NONE; // hidden; the value is stored in overrides
        }
        p_property.hint_string = type == TYPE_SLIDE ? "-10,10,0.001,or_less,or_greater,suffix:m" : "-360,360,0.1,or_less,or_greater,radians_as_degrees";
    } else if ((name == StringName("limited") || name == StringName("stiffness") || String(name).begins_with("limit_")) && !has_axis) {
        p_property.usage = PROPERTY_USAGE_NONE; // hidden; the value is stored in overrides
    } else if (name == StringName("range")) {
        if (!has_axis || !is_limited()) {
            p_property.usage = PROPERTY_USAGE_NONE; // hidden; the value is stored in overrides
        }
        // Stored in radians and shown in degrees, like Node3D.rotation.
        p_property.hint_string = type == TYPE_SLIDE ? "-10,10,0.001,or_less,or_greater,suffix:m" : "-360,360,0.1,or_less,or_greater,radians_as_degrees";
    }
}

void MujocoJoint::_bind_methods()
{
    BIND_ENUM_CONSTANT(TYPE_FREE);
    BIND_ENUM_CONSTANT(TYPE_BALL);
    BIND_ENUM_CONSTANT(TYPE_SLIDE);
    BIND_ENUM_CONSTANT(TYPE_HINGE);

    ClassDB::bind_method(D_METHOD("set_type", "type"), &MujocoJoint::set_type);
    ClassDB::bind_method(D_METHOD("get_type"), &MujocoJoint::get_type);
    ClassDB::bind_method(D_METHOD("set_limited", "limited"), &MujocoJoint::set_limited);
    ClassDB::bind_method(D_METHOD("is_limited"), &MujocoJoint::is_limited);
    ClassDB::bind_method(D_METHOD("set_range", "range"), &MujocoJoint::set_range);
    ClassDB::bind_method(D_METHOD("get_range"), &MujocoJoint::get_range);
    ClassDB::bind_method(D_METHOD("set_damping", "damping"), &MujocoJoint::set_damping);
    ClassDB::bind_method(D_METHOD("get_damping"), &MujocoJoint::get_damping);
    ClassDB::bind_method(D_METHOD("set_stiffness", "stiffness"), &MujocoJoint::set_stiffness);
    ClassDB::bind_method(D_METHOD("get_stiffness"), &MujocoJoint::get_stiffness);
    ClassDB::bind_method(D_METHOD("set_armature", "armature"), &MujocoJoint::set_armature);
    ClassDB::bind_method(D_METHOD("get_armature"), &MujocoJoint::get_armature);
    ClassDB::bind_method(D_METHOD("set_frictionloss", "frictionloss"), &MujocoJoint::set_frictionloss);
    ClassDB::bind_method(D_METHOD("get_frictionloss"), &MujocoJoint::get_frictionloss);
    ClassDB::bind_method(D_METHOD("set_ref", "ref"), &MujocoJoint::set_ref);
    ClassDB::bind_method(D_METHOD("get_ref"), &MujocoJoint::get_ref);
    ClassDB::bind_method(D_METHOD("set_springref", "springref"), &MujocoJoint::set_springref);
    ClassDB::bind_method(D_METHOD("get_springref"), &MujocoJoint::get_springref);
#define JOINT_BIND_NUMBER(m_name) \
    ClassDB::bind_method(D_METHOD("set_" #m_name, "value"), &MujocoJoint::set_##m_name); \
    ClassDB::bind_method(D_METHOD("get_" #m_name), &MujocoJoint::get_##m_name);
    JOINT_BIND_NUMBER(limit_margin)
    JOINT_BIND_NUMBER(limit_solref_timeconst)
    JOINT_BIND_NUMBER(limit_solref_dampratio)
    JOINT_BIND_NUMBER(limit_solimp_dmin)
    JOINT_BIND_NUMBER(limit_solimp_dmax)
    JOINT_BIND_NUMBER(limit_solimp_width)
    JOINT_BIND_NUMBER(limit_solimp_midpoint)
    JOINT_BIND_NUMBER(limit_solimp_power)
#undef JOINT_BIND_NUMBER
    ClassDB::bind_method(D_METHOD("get_joint_position"), &MujocoJoint::get_joint_position);
    ClassDB::bind_method(D_METHOD("set_joint_position", "position"), &MujocoJoint::set_joint_position);
    ClassDB::bind_method(D_METHOD("get_joint_velocity"), &MujocoJoint::get_joint_velocity);
    ClassDB::bind_method(D_METHOD("set_joint_velocity", "velocity"), &MujocoJoint::set_joint_velocity);

    // Attribute properties are stored in "overrides", so they are editor-only here.
    ADD_PROPERTY(PropertyInfo(Variant::INT, "type", PROPERTY_HINT_ENUM, "Free:0,Ball:1,Slide:2,Hinge:3", PROPERTY_USAGE_EDITOR), "set_type", "get_type");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "limited", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR), "set_limited", "is_limited");
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "range", PROPERTY_HINT_RANGE, "-360,360,0.1,or_less,or_greater,radians_as_degrees", PROPERTY_USAGE_EDITOR), "set_range", "get_range");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "damping", PROPERTY_HINT_RANGE, "0,100,0.001,or_greater", PROPERTY_USAGE_EDITOR), "set_damping", "get_damping");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "stiffness", PROPERTY_HINT_RANGE, "0,1000,0.001,or_greater", PROPERTY_USAGE_EDITOR), "set_stiffness", "get_stiffness");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "armature", PROPERTY_HINT_RANGE, "0,10,0.0001,or_greater", PROPERTY_USAGE_EDITOR), "set_armature", "get_armature");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "frictionloss", PROPERTY_HINT_RANGE, "0,100,0.001,or_greater", PROPERTY_USAGE_EDITOR), "set_frictionloss", "get_frictionloss");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "ref", PROPERTY_HINT_RANGE, "-360,360,0.1,or_less,or_greater,radians_as_degrees", PROPERTY_USAGE_EDITOR), "set_ref", "get_ref");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "springref", PROPERTY_HINT_RANGE, "-360,360,0.1,or_less,or_greater,radians_as_degrees", PROPERTY_USAGE_EDITOR), "set_springref", "get_springref");

    ADD_GROUP("Limit", "limit_");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "limit_margin", PROPERTY_HINT_RANGE, "0,1,0.0001,or_greater", PROPERTY_USAGE_EDITOR), "set_limit_margin", "get_limit_margin");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "limit_solref_timeconst", PROPERTY_HINT_RANGE, "0.0001,1,0.0001,or_greater,or_less,suffix:s", PROPERTY_USAGE_EDITOR), "set_limit_solref_timeconst", "get_limit_solref_timeconst");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "limit_solref_dampratio", PROPERTY_HINT_RANGE, "0,2,0.001,or_greater,or_less", PROPERTY_USAGE_EDITOR), "set_limit_solref_dampratio", "get_limit_solref_dampratio");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "limit_solimp_dmin", PROPERTY_HINT_RANGE, "0.0001,0.9999,0.0001", PROPERTY_USAGE_EDITOR), "set_limit_solimp_dmin", "get_limit_solimp_dmin");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "limit_solimp_dmax", PROPERTY_HINT_RANGE, "0.0001,0.9999,0.0001", PROPERTY_USAGE_EDITOR), "set_limit_solimp_dmax", "get_limit_solimp_dmax");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "limit_solimp_width", PROPERTY_HINT_RANGE, "0,1,0.0001,or_greater", PROPERTY_USAGE_EDITOR), "set_limit_solimp_width", "get_limit_solimp_width");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "limit_solimp_midpoint", PROPERTY_HINT_RANGE, "0,1,0.001", PROPERTY_USAGE_EDITOR), "set_limit_solimp_midpoint", "get_limit_solimp_midpoint");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "limit_solimp_power", PROPERTY_HINT_RANGE, "1,10,0.01,or_greater", PROPERTY_USAGE_EDITOR), "set_limit_solimp_power", "get_limit_solimp_power");
}

void MujocoJoint::set_type(Type p_type)
{
    ERR_FAIL_COND_MSG(p_type < TYPE_FREE || p_type > TYPE_HINGE, vformat("Unsupported joint type: %d.", p_type));
    set_attribute("type", p_type);
    notify_property_list_changed();
    changed();
}

MujocoJoint::Type MujocoJoint::get_type() const { return static_cast<Type>(static_cast<int64_t>(get_attribute("type"))); }

void MujocoJoint::set_limited(bool p_limited)
{
    set_attribute("limited", p_limited);
    notify_property_list_changed();
    changed();
}

bool MujocoJoint::is_limited() const { return get_attribute("limited"); }

void MujocoJoint::set_range(const Vector2 &p_range)
{
    set_attribute("range", p_range);
    changed();
}

Vector2 MujocoJoint::get_range() const { return get_attribute("range"); }

void MujocoJoint::set_damping(double p_damping)
{
    ERR_FAIL_COND_MSG(p_damping < 0.0, "Damping must not be negative.");
    set_attribute("damping", p_damping);
    changed();
}

double MujocoJoint::get_damping() const { return get_attribute("damping"); }

void MujocoJoint::set_stiffness(double p_stiffness)
{
    ERR_FAIL_COND_MSG(p_stiffness < 0.0, "Stiffness must not be negative.");
    set_attribute("stiffness", p_stiffness);
    changed();
}

double MujocoJoint::get_stiffness() const { return get_attribute("stiffness"); }

void MujocoJoint::set_armature(double p_armature)
{
    ERR_FAIL_COND_MSG(p_armature < 0.0, "Armature must not be negative.");
    set_attribute("armature", p_armature);
    changed();
}

double MujocoJoint::get_armature() const { return get_attribute("armature"); }

void MujocoJoint::set_frictionloss(double p_frictionloss)
{
    ERR_FAIL_COND_MSG(p_frictionloss < 0.0, "Friction loss must not be negative.");
    set_attribute("frictionloss", p_frictionloss);
    changed();
}

double MujocoJoint::get_frictionloss() const { return get_attribute("frictionloss"); }

void MujocoJoint::set_ref(double p_ref)
{
    set_attribute("ref", p_ref);
    changed();
}

double MujocoJoint::get_ref() const { return get_attribute("ref"); }

void MujocoJoint::set_springref(double p_springref)
{
    set_attribute("springref", p_springref);
    changed();
}

double MujocoJoint::get_springref() const { return get_attribute("springref"); }

// The limit settings are plain numbers, checked by MuJoCo's compiler.
#define JOINT_NUMBER_ATTRIBUTE(m_name) \
    void MujocoJoint::set_##m_name(double p_value) \
    { \
        set_attribute(#m_name, p_value); \
        changed(); \
    } \
    double MujocoJoint::get_##m_name() const { return get_attribute(#m_name); }

JOINT_NUMBER_ATTRIBUTE(limit_margin)
JOINT_NUMBER_ATTRIBUTE(limit_solref_timeconst)
JOINT_NUMBER_ATTRIBUTE(limit_solref_dampratio)
JOINT_NUMBER_ATTRIBUTE(limit_solimp_dmin)
JOINT_NUMBER_ATTRIBUTE(limit_solimp_dmax)
JOINT_NUMBER_ATTRIBUTE(limit_solimp_width)
JOINT_NUMBER_ATTRIBUTE(limit_solimp_midpoint)
JOINT_NUMBER_ATTRIBUTE(limit_solimp_power)

#undef JOINT_NUMBER_ATTRIBUTE
