#include "mujoco_joint.h"
#include "mujoco_body.h"
#include "mujoco_convert.h"
#include "mujoco_world.h"

using namespace godot;

mjsBody *MujocoJoint::add_to_spec(mjSpec * /*p_spec*/, mjsBody *p_body, const Transform3D &p_transform, const String &p_name)
{
    if (type == TYPE_FREE) {
        // A free joint has no anchor or axis.
        mjsJoint *joint = mjs_addFreeJoint(p_body);
        mjs_setName(joint->element, p_name.utf8().get_data());
        return p_body;
    }

    mjsJoint *joint = mjs_addJoint(p_body, nullptr);
    mjs_setName(joint->element, p_name.utf8().get_data());
    joint->type = static_cast<mjtJoint>(type);

    mjtNum quat_unused[4];
    mujoco_convert::to_mujoco(p_transform, joint->pos, quat_unused);
    // The node's local Y axis, expressed in the body's frame.
    mujoco_convert::to_mujoco(p_transform.basis.xform(Vector3(0, 1, 0)).normalized(), joint->axis);

    // The spec's compiler uses degrees (MuJoCo's default), matching the range units.
    joint->limited = limited ? mjLIMITED_TRUE : mjLIMITED_FALSE;
    joint->range[0] = type == TYPE_BALL ? 0.0 : range.x;
    joint->range[1] = range.y;
    joint->damping[0] = damping;
    joint->stiffness[0] = stiffness;
    joint->armature = armature;
    joint->frictionloss = frictionloss;
    return p_body;
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
    if (limited && type != TYPE_FREE && type != TYPE_BALL && range.x >= range.y) {
        warnings.push_back("Range minimum must be less than its maximum.");
    }
    return warnings;
}

void MujocoJoint::changed()
{
    update_configuration_warnings();
    request_compile();
}

void MujocoJoint::_validate_property(PropertyInfo &p_property) const
{
    // Free joints have no axis, so no limits or spring.
    const bool has_axis = type != TYPE_FREE;
    const StringName name = p_property.name;

    if ((name == StringName("limited") || name == StringName("stiffness")) && !has_axis) {
        p_property.usage = PROPERTY_USAGE_NO_EDITOR;
    } else if (name == StringName("range")) {
        if (!has_axis || !limited) {
            p_property.usage = PROPERTY_USAGE_NO_EDITOR;
        }
        p_property.hint_string = type == TYPE_SLIDE ? "suffix:m" : "suffix:deg";
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

    ADD_PROPERTY(PropertyInfo(Variant::INT, "type", PROPERTY_HINT_ENUM, "Free:0,Ball:1,Slide:2,Hinge:3"), "set_type", "get_type");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "limited"), "set_limited", "is_limited");
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "range"), "set_range", "get_range");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "damping", PROPERTY_HINT_RANGE, "0,100,0.001,or_greater"), "set_damping", "get_damping");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "stiffness", PROPERTY_HINT_RANGE, "0,1000,0.001,or_greater"), "set_stiffness", "get_stiffness");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "armature", PROPERTY_HINT_RANGE, "0,10,0.0001,or_greater"), "set_armature", "get_armature");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "frictionloss", PROPERTY_HINT_RANGE, "0,100,0.001,or_greater"), "set_frictionloss", "get_frictionloss");
}

void MujocoJoint::set_type(Type p_type)
{
    ERR_FAIL_COND_MSG(p_type < TYPE_FREE || p_type > TYPE_HINGE, vformat("Unsupported joint type: %d.", p_type));
    type = p_type;
    notify_property_list_changed();
    changed();
}

MujocoJoint::Type MujocoJoint::get_type() const { return type; }

void MujocoJoint::set_limited(bool p_limited)
{
    limited = p_limited;
    notify_property_list_changed();
    changed();
}

bool MujocoJoint::is_limited() const { return limited; }

void MujocoJoint::set_range(const Vector2 &p_range)
{
    range = p_range;
    changed();
}

Vector2 MujocoJoint::get_range() const { return range; }

void MujocoJoint::set_damping(double p_damping)
{
    ERR_FAIL_COND_MSG(p_damping < 0.0, "Damping must not be negative.");
    damping = p_damping;
    changed();
}

double MujocoJoint::get_damping() const { return damping; }

void MujocoJoint::set_stiffness(double p_stiffness)
{
    ERR_FAIL_COND_MSG(p_stiffness < 0.0, "Stiffness must not be negative.");
    stiffness = p_stiffness;
    changed();
}

double MujocoJoint::get_stiffness() const { return stiffness; }

void MujocoJoint::set_armature(double p_armature)
{
    ERR_FAIL_COND_MSG(p_armature < 0.0, "Armature must not be negative.");
    armature = p_armature;
    changed();
}

double MujocoJoint::get_armature() const { return armature; }

void MujocoJoint::set_frictionloss(double p_frictionloss)
{
    ERR_FAIL_COND_MSG(p_frictionloss < 0.0, "Friction loss must not be negative.");
    frictionloss = p_frictionloss;
    changed();
}

double MujocoJoint::get_frictionloss() const { return frictionloss; }
