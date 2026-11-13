#include "mujoco_joint.h"

#include "mujoco_convert.h"

using namespace godot;

namespace {

// Indices into ATTRIBUTES below. The two must stay in the same order.
enum Attribute {
    TYPE,
    ATTRIBUTE_COUNT,
};

const MujocoAttribute ATTRIBUTES[] = {
    { "type", Variant::INT, PROPERTY_HINT_ENUM, "Free:0,Ball:1,Slide:2,Hinge:3" },
};

static_assert(sizeof(ATTRIBUTES) / sizeof(ATTRIBUTES[0]) == ATTRIBUTE_COUNT, "ATTRIBUTES and enum Attribute must match");
static_assert(mjJNT_FREE == 0 && mjJNT_BALL == 1 && mjJNT_SLIDE == 2 && mjJNT_HINGE == 3);

} // namespace

void MujocoJoint::_bind_methods()
{
    BIND_ENUM_CONSTANT(TYPE_FREE);
    BIND_ENUM_CONSTANT(TYPE_BALL);
    BIND_ENUM_CONSTANT(TYPE_SLIDE);
    BIND_ENUM_CONSTANT(TYPE_HINGE);
}

// Lists the attributes here, so the inspector shows them under "MujocoJoint".
void MujocoJoint::_get_property_list(List<PropertyInfo> *r_list) const
{
    get_attribute_property_list(r_list);
}

const MujocoAttribute *MujocoJoint::get_attribute_table(int &r_count) const
{
    r_count = ATTRIBUTE_COUNT;
    return ATTRIBUTES;
}

Variant MujocoJoint::read(const mjsJoint &p_joint, int p_index)
{
    switch (p_index) {
        case TYPE:
            return int(p_joint.type);
    }
    return Variant();
}

void MujocoJoint::write(mjsJoint &r_joint, int p_index, const Variant &p_value)
{
    switch (p_index) {
        case TYPE:
            r_joint.type = mjtJoint(int(p_value));
            break;
    }
}

// MuJoCo's built-in defaults. Note the default type is a hinge, as in MJCF.
Variant MujocoJoint::get_inherited_value(int p_index) const
{
    mjsJoint defaults;
    mjs_defaultJoint(&defaults);
    return read(defaults, p_index);
}

mjsBody *MujocoJoint::add_to_spec(mjSpec *p_spec, mjsBody *p_body, const Transform3D &p_transform) const
{
    mjsJoint *joint = mjs_addJoint(p_body, nullptr);
    mjs_setName(joint->element, get_mujoco_name().utf8().get_data());
    // The node's position is the joint's anchor point in its body.
    mujoco_convert::to_mujoco(p_transform.origin, joint->pos);

    for (int i = 0; i < ATTRIBUTE_COUNT; i++) {
        Variant value;
        if (get_override(i, value)) {
            write(*joint, i, value);
        }
    }

    // A joint can't contain other elements.
    return nullptr;
}
