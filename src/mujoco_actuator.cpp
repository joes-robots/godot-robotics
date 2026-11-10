#include "mujoco_actuator.h"
#include "mujoco_joint.h"
#include "mujoco_world.h"

#include <godot_cpp/classes/engine.hpp>

using namespace godot;

namespace {

const MujocoAttribute ACTUATOR_ATTRIBUTES[] = {
    { "gear", Variant::FLOAT, { 1.0 } },
    { "kp", Variant::FLOAT, { 1.0 } },
    { "kv", Variant::FLOAT, { 0.0 } },
    { "ctrl_limited", Variant::BOOL, { 0 } },
    { "ctrl_range", Variant::VECTOR2, { -1, 1 } },
    { "force_limited", Variant::BOOL, { 0 } },
    { "force_range", Variant::VECTOR2, { -1, 1 } },
};
const MujocoAttributeTable ACTUATOR_TABLE = MUJOCO_ATTRIBUTE_TABLE(ACTUATOR_ATTRIBUTES);

} // namespace

const MujocoAttributeTable *MujocoActuator::get_attribute_table() const
{
    return &ACTUATOR_TABLE;
}

void MujocoActuator::overrides_changed()
{
    notify_property_list_changed();
    changed();
}

MujocoJoint *MujocoActuator::find_joint(String &r_problem) const
{
    const MujocoWorld *world = get_world();
    if (!world) {
        r_problem = "Not inside a MujocoWorld.";
        return nullptr;
    }
    MujocoJoint *target = Object::cast_to<MujocoJoint>(get_node_or_null(joint));
    if (!target) {
        r_problem = "\"joint\" must point to a MujocoJoint. By default (\"..\") it is the parent node.";
        return nullptr;
    }
    if (target->get_world() != world) {
        r_problem = "The joint belongs to a different MujocoWorld.";
        return nullptr;
    }
    if (target->get_type() != MujocoJoint::TYPE_HINGE && target->get_type() != MujocoJoint::TYPE_SLIDE) {
        r_problem = "Only hinge and slide joints can be actuated.";
        return nullptr;
    }
    return target;
}

mjsBody *MujocoActuator::add_to_spec(mjSpec *p_spec, mjsBody *p_body, const Transform3D & /*p_transform*/, const String &p_name)
{
    String problem;
    MujocoJoint *target = find_joint(problem);
    if (!target) {
        // Leave it out rather than fail the whole model. The editor shows the
        // problem as a configuration warning; at runtime, say so once per compile.
        if (!Engine::get_singleton()->is_editor_hint()) {
            WARN_PRINT(vformat("MuJoCo actuator '%s' is left out of the model: %s", p_name, problem));
        }
        return p_body;
    }

    mjsActuator *actuator = mjs_addActuator(p_spec, nullptr);
    mjs_setName(actuator->element, p_name.utf8().get_data());
    actuator->trntype = mjTRN_JOINT;
    // Joint names are their node paths relative to the world, like this actuator's.
    mjs_setString(actuator->target, String(get_world()->get_path_to(target)).utf8().get_data());
    actuator->gear[0] = get_gear();
    const double kp = get_kp();
    const double kv = get_kv();

    // MuJoCo's documented expansions of the <motor>, <position> and <velocity> shortcuts:
    // force = gain * ctrl + bias0 + bias1 * position + bias2 * velocity. Set directly
    // rather than through mjs_setToPosition, whose optional arguments use sentinels.
    actuator->dyntype = mjDYN_NONE;
    actuator->gaintype = mjGAIN_FIXED;
    mju_zero(actuator->gainprm, mjNGAIN);
    mju_zero(actuator->biasprm, mjNBIAS);
    switch (type) {
        case TYPE_MOTOR:
            actuator->gainprm[0] = 1.0;
            actuator->biastype = mjBIAS_NONE;
            break;
        case TYPE_POSITION:
            actuator->gainprm[0] = kp;
            actuator->biastype = mjBIAS_AFFINE;
            actuator->biasprm[1] = -kp;
            actuator->biasprm[2] = -kv;
            break;
        case TYPE_VELOCITY:
            actuator->gainprm[0] = kv;
            actuator->biastype = mjBIAS_AFFINE;
            actuator->biasprm[2] = -kv;
            break;
    }

    const Vector2 ctrl_range = get_ctrl_range();
    const Vector2 force_range = get_force_range();
    actuator->ctrllimited = is_ctrl_limited() ? mjLIMITED_TRUE : mjLIMITED_FALSE;
    actuator->ctrlrange[0] = ctrl_range.x;
    actuator->ctrlrange[1] = ctrl_range.y;
    actuator->forcelimited = is_force_limited() ? mjLIMITED_TRUE : mjLIMITED_FALSE;
    actuator->forcerange[0] = force_range.x;
    actuator->forcerange[1] = force_range.y;
    return p_body;
}

PackedStringArray MujocoActuator::_get_configuration_warnings() const
{
    PackedStringArray warnings = MujocoElement::_get_configuration_warnings();
    String problem;
    if (get_world() && !find_joint(problem)) {
        warnings.push_back(problem + " This actuator is left out of the model.");
    }
    if (is_ctrl_limited() && get_ctrl_range().x >= get_ctrl_range().y) {
        warnings.push_back("Control range minimum must be less than its maximum.");
    }
    if (is_force_limited() && get_force_range().x >= get_force_range().y) {
        warnings.push_back("Force range minimum must be less than its maximum.");
    }
    if (type == TYPE_VELOCITY && get_kv() == 0.0) {
        warnings.push_back("A velocity actuator with kv = 0 has no effect.");
    }
    return warnings;
}

int MujocoActuator::get_actuator_id()
{
    return get_model_id(mjOBJ_ACTUATOR);
}

void MujocoActuator::write_ctrl()
{
    const int id = get_actuator_id();
    if (id < 0) {
        return;
    }
    // Takes effect at the next step (from a pre_step handler: the step about to run).
    get_world()->get_data()->ctrl[id] = ctrl;
}

double MujocoActuator::get_force()
{
    const int id = get_actuator_id();
    ERR_FAIL_COND_V_MSG(id < 0, 0.0, "This actuator is not part of a compiled MuJoCo model.");
    return get_world()->get_data()->actuator_force[id];
}

void MujocoActuator::changed()
{
    update_configuration_warnings();
    request_compile();
}

void MujocoActuator::_validate_property(PropertyInfo &p_property) const
{
    const StringName name = p_property.name;
    if (name == StringName("kp") && type != TYPE_POSITION) {
        p_property.usage = PROPERTY_USAGE_NONE; // hidden; the value is stored in overrides
    } else if (name == StringName("kv") && type == TYPE_MOTOR) {
        p_property.usage = PROPERTY_USAGE_NONE; // hidden; the value is stored in overrides
    } else if (name == StringName("ctrl_range") && !is_ctrl_limited()) {
        p_property.usage = PROPERTY_USAGE_NONE; // hidden; the value is stored in overrides
    } else if (name == StringName("force_range") && !is_force_limited()) {
        p_property.usage = PROPERTY_USAGE_NONE; // hidden; the value is stored in overrides
    }
}

void MujocoActuator::_bind_methods()
{
    BIND_ENUM_CONSTANT(TYPE_MOTOR);
    BIND_ENUM_CONSTANT(TYPE_POSITION);
    BIND_ENUM_CONSTANT(TYPE_VELOCITY);

    ClassDB::bind_method(D_METHOD("set_type", "type"), &MujocoActuator::set_type);
    ClassDB::bind_method(D_METHOD("get_type"), &MujocoActuator::get_type);
    ClassDB::bind_method(D_METHOD("set_joint", "joint"), &MujocoActuator::set_joint);
    ClassDB::bind_method(D_METHOD("get_joint"), &MujocoActuator::get_joint);
    ClassDB::bind_method(D_METHOD("set_ctrl", "ctrl"), &MujocoActuator::set_ctrl);
    ClassDB::bind_method(D_METHOD("get_ctrl"), &MujocoActuator::get_ctrl);
    ClassDB::bind_method(D_METHOD("set_gear", "gear"), &MujocoActuator::set_gear);
    ClassDB::bind_method(D_METHOD("get_gear"), &MujocoActuator::get_gear);
    ClassDB::bind_method(D_METHOD("set_kp", "kp"), &MujocoActuator::set_kp);
    ClassDB::bind_method(D_METHOD("get_kp"), &MujocoActuator::get_kp);
    ClassDB::bind_method(D_METHOD("set_kv", "kv"), &MujocoActuator::set_kv);
    ClassDB::bind_method(D_METHOD("get_kv"), &MujocoActuator::get_kv);
    ClassDB::bind_method(D_METHOD("set_ctrl_limited", "limited"), &MujocoActuator::set_ctrl_limited);
    ClassDB::bind_method(D_METHOD("is_ctrl_limited"), &MujocoActuator::is_ctrl_limited);
    ClassDB::bind_method(D_METHOD("set_ctrl_range", "range"), &MujocoActuator::set_ctrl_range);
    ClassDB::bind_method(D_METHOD("get_ctrl_range"), &MujocoActuator::get_ctrl_range);
    ClassDB::bind_method(D_METHOD("set_force_limited", "limited"), &MujocoActuator::set_force_limited);
    ClassDB::bind_method(D_METHOD("is_force_limited"), &MujocoActuator::is_force_limited);
    ClassDB::bind_method(D_METHOD("set_force_range", "range"), &MujocoActuator::set_force_range);
    ClassDB::bind_method(D_METHOD("get_force_range"), &MujocoActuator::get_force_range);
    ClassDB::bind_method(D_METHOD("get_force"), &MujocoActuator::get_force);

    ADD_PROPERTY(PropertyInfo(Variant::INT, "type", PROPERTY_HINT_ENUM, "Motor,Position,Velocity"), "set_type", "get_type");
    ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "joint", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "MujocoJoint"), "set_joint", "get_joint");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "ctrl", PROPERTY_HINT_RANGE, "-100,100,0.001,or_less,or_greater"), "set_ctrl", "get_ctrl");
    // Attribute properties are stored in "overrides", so they are editor-only here.
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "gear", PROPERTY_HINT_RANGE, "-100,100,0.001,or_less,or_greater", PROPERTY_USAGE_EDITOR), "set_gear", "get_gear");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "kp", PROPERTY_HINT_RANGE, "0,1000,0.001,or_greater", PROPERTY_USAGE_EDITOR), "set_kp", "get_kp");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "kv", PROPERTY_HINT_RANGE, "0,100,0.001,or_greater", PROPERTY_USAGE_EDITOR), "set_kv", "get_kv");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "ctrl_limited", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR), "set_ctrl_limited", "is_ctrl_limited");
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "ctrl_range", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR), "set_ctrl_range", "get_ctrl_range");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "force_limited", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR), "set_force_limited", "is_force_limited");
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "force_range", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR), "set_force_range", "get_force_range");
}

void MujocoActuator::set_type(Type p_type)
{
    ERR_FAIL_COND_MSG(p_type < TYPE_MOTOR || p_type > TYPE_VELOCITY, vformat("Unsupported actuator type: %d.", p_type));
    type = p_type;
    notify_property_list_changed();
    changed();
}

MujocoActuator::Type MujocoActuator::get_type() const { return type; }

void MujocoActuator::set_joint(const NodePath &p_joint)
{
    joint = p_joint;
    changed();
}

NodePath MujocoActuator::get_joint() const { return joint; }

void MujocoActuator::set_ctrl(double p_ctrl)
{
    // Not part of the model: no recompile.
    ctrl = p_ctrl;
    write_ctrl();
}

double MujocoActuator::get_ctrl() const { return ctrl; }

void MujocoActuator::set_gear(double p_gear)
{
    set_attribute("gear", p_gear);
    changed();
}

double MujocoActuator::get_gear() const { return get_attribute("gear"); }

void MujocoActuator::set_kp(double p_kp)
{
    ERR_FAIL_COND_MSG(p_kp < 0.0, "kp must not be negative.");
    set_attribute("kp", p_kp);
    changed();
}

double MujocoActuator::get_kp() const { return get_attribute("kp"); }

void MujocoActuator::set_kv(double p_kv)
{
    ERR_FAIL_COND_MSG(p_kv < 0.0, "kv must not be negative.");
    set_attribute("kv", p_kv);
    changed();
}

double MujocoActuator::get_kv() const { return get_attribute("kv"); }

void MujocoActuator::set_ctrl_limited(bool p_limited)
{
    set_attribute("ctrl_limited", p_limited);
    notify_property_list_changed();
    changed();
}

bool MujocoActuator::is_ctrl_limited() const { return get_attribute("ctrl_limited"); }

void MujocoActuator::set_ctrl_range(const Vector2 &p_range)
{
    set_attribute("ctrl_range", p_range);
    changed();
}

Vector2 MujocoActuator::get_ctrl_range() const { return get_attribute("ctrl_range"); }

void MujocoActuator::set_force_limited(bool p_limited)
{
    set_attribute("force_limited", p_limited);
    notify_property_list_changed();
    changed();
}

bool MujocoActuator::is_force_limited() const { return get_attribute("force_limited"); }

void MujocoActuator::set_force_range(const Vector2 &p_range)
{
    set_attribute("force_range", p_range);
    changed();
}

Vector2 MujocoActuator::get_force_range() const { return get_attribute("force_range"); }
