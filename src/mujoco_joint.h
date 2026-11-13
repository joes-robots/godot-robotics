#pragma once

#include "mujoco_element.h"

// A MuJoCo joint (<joint> in MJCF): what lets a body move relative to its parent.
//
// A joint node is a child of the MujocoBody it moves. A body with no joint is
// welded to its parent. A free joint lets the body move without any constraint
// and is only allowed on bodies directly under the world.
//
// The node's position is the joint's anchor: the point the body turns about.
//
// Attributes (see MujocoElement for how they are stored). Angles are in
// radians; the inspector shows them in degrees.
//
//   type          one of the TYPE_ constants. MuJoCo's default is a hinge.
//   axis          hinge: the axis turned about. slide: the direction slid along.
//                 Given in this node's own frame, so rotating the node turns
//                 the axis with it. Default (0, 1, 0), which is MuJoCo's
//                 default (0, 0, 1) in Godot's convention.
//   limited       whether the joint's travel is limited. Off by default.
//   range_lower, range_upper
//                 the limits, used while `limited` is on. hinge: angles.
//                 slide: metres. ball: range_upper is the largest angle away
//                 from the reference pose, and there is no lower limit.
//   ref           the joint's value in the pose the model is built in.
//   springref     the value at which the joint's spring is at rest.
//   stiffness     spring stiffness, pulling the joint towards springref.
//   damping       resistance proportional to joint velocity.
//   armature      extra inertia on the joint, as from a geared motor's rotor.
//   frictionloss  dry friction.
//
// An attribute that means nothing for the current type (the axis of a free
// joint, say) is greyed out in the inspector. Its value is kept, and not used.
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
    mjtObj get_mujoco_type() const override { return mjOBJ_JOINT; }

    // The joint's state in the running simulation, for hinge and slide joints,
    // which have a single number each: an angle in radians, or a distance in
    // metres, and its rate of change. Ball and free joints have no single
    // number; use the world's qpos and qvel arrays for those, with the
    // addresses below.
    //
    // Setting either takes effect at once: bodies move to match. To set many
    // joints at a time, MujocoWorld.set_qpos is cheaper.
    double get_joint_position() const;
    void set_joint_position(double p_position);
    double get_joint_velocity() const;
    void set_joint_velocity(double p_velocity);

    // A force (slide, in N) or torque (hinge, in N m) applied to the joint from
    // outside. It stays in effect until changed; reset() and recompiling clear
    // it. Hinge and slide joints only.
    double get_applied_force() const;
    void set_applied_force(double p_force);

    // Where this joint's values start in the world's qpos and qvel arrays, or
    // -1 if it isn't in the compiled model. A hinge or slide has one entry in
    // each; a ball has 4 in qpos and 3 in qvel; a free joint has 7 and 6.
    int get_qpos_address() const;
    int get_qvel_address() const;

    // How long to draw this joint's editor gizmo, in metres: long enough to
    // stick out of the geoms on the same body.
    float get_gizmo_size() const;

protected:
    static void _bind_methods();
    void _notification(int p_what);
    void _validate_property(godot::PropertyInfo &p_property) const;
    void attributes_changed() override;

    void _get_property_list(godot::List<godot::PropertyInfo> *r_list) const;

    const MujocoAttribute *get_attribute_table(int &r_count) const override;
    godot::Variant get_inherited_value(int p_index) const override;
    void customize_attribute_property(int p_index, godot::PropertyInfo &r_property) const override;
    bool changes_attribute_presentation(int p_index) const override;
    bool write_live(int p_index, MujocoWorld &p_world) const override;

private:
    // True for the joint types whose state is a single number.
    bool has_scalar_state() const;
    // The world, if this joint is in its compiled model; otherwise nullptr.
    MujocoWorld *get_simulating_world() const;

    // Whether attribute p_index means anything for this joint's current type.
    bool applies(int p_index) const;
    static godot::Variant read(const mjsJoint &p_joint, int p_index);
    static void write(mjsJoint &r_joint, int p_index, const godot::Variant &p_value);
};

VARIANT_ENUM_CAST(MujocoJoint::Type);
