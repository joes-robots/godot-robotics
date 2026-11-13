#pragma once

#include "mujoco_element.h"

// A MuJoCo joint (<joint> in MJCF): what lets a body move relative to its parent.
//
// A joint node is a child of the MujocoBody it moves. A body with no joint is
// welded to its parent. A free joint lets the body move without any constraint
// and is only allowed on bodies directly under the world.
//
// This is the minimal version: it only has a type. Axes, ranges, damping and
// the rest come in a later milestone.
class MujocoJoint : public MujocoElement {
    GDCLASS(MujocoJoint, MujocoElement);

public:
    // The values are MuJoCo's own (mjtJoint).
    enum Type {
        TYPE_FREE = mjJNT_FREE,
        TYPE_BALL = mjJNT_BALL,
        TYPE_SLIDE = mjJNT_SLIDE,
        TYPE_HINGE = mjJNT_HINGE,
    };

    mjsBody *add_to_spec(mjSpec *p_spec, mjsBody *p_body, const godot::Transform3D &p_transform) const override;

protected:
    static void _bind_methods();

    void _get_property_list(godot::List<godot::PropertyInfo> *r_list) const;

    const MujocoAttribute *get_attribute_table(int &r_count) const override;
    godot::Variant get_inherited_value(int p_index) const override;

private:
    static godot::Variant read(const mjsJoint &p_joint, int p_index);
    static void write(mjsJoint &r_joint, int p_index, const godot::Variant &p_value);
};

VARIANT_ENUM_CAST(MujocoJoint::Type);
