#pragma once

#include "mujoco_element.h"

#include <godot_cpp/variant/vector2.hpp>

// A MuJoCo joint: gives the body it belongs to (the nearest MujocoBody
// ancestor) degrees of freedom relative to that body's parent.
//
// The node's position is the joint anchor and its local Y axis is the hinge or
// slide axis (MuJoCo's default joint axis, local z, converted to Godot). Free
// joints ignore both and must belong to a body whose parent is the world.
//
// The pose in the scene is joint position `ref` (0 unless set; MJCF's ref), so
// by default positions are measured from it. A joint's spring (stiffness) pulls
// toward springref.
// Positive hinge rotation follows the right-hand rule about the axis.
//
// The Limit group sets how the limits behave when hit (MJCF's margin, solreflimit
// and solimplimit; named like MujocoGeom's contact solver settings): the limit
// acts within limit_margin of the range, and is softer with a larger
// limit_solref_timeconst.
class MujocoJoint : public MujocoElement {
    GDCLASS(MujocoJoint, MujocoElement);

public:
    // Values are MuJoCo's mjtJoint.
    enum Type {
        TYPE_FREE = mjJNT_FREE,
        TYPE_BALL = mjJNT_BALL,
        TYPE_SLIDE = mjJNT_SLIDE,
        TYPE_HINGE = mjJNT_HINGE,
    };

    mjsBody *add_to_spec(mjSpec *p_spec, mjsBody *p_body, const godot::Transform3D &p_transform, const godot::String &p_name) override;
    const char *get_class_section() const override { return "joint"; }
    void write_class_defaults(mjsDefault *p_default) override;
    void inherited_changed() override;
    godot::PackedStringArray _get_configuration_warnings() const override;

    void set_type(Type p_type);
    Type get_type() const;
    void set_limited(bool p_limited);
    bool is_limited() const;
    void set_range(const godot::Vector2 &p_range);
    godot::Vector2 get_range() const;
    void set_damping(double p_damping);
    double get_damping() const;
    void set_stiffness(double p_stiffness);
    double get_stiffness() const;
    void set_armature(double p_armature);
    double get_armature() const;
    void set_frictionloss(double p_frictionloss);
    double get_frictionloss() const;
    // Hinge and slide joints: the position of the scene pose, and the spring's rest
    // position (radians or meters).
    void set_ref(double p_ref);
    double get_ref() const;
    void set_springref(double p_springref);
    double get_springref() const;
    // A cap on the total force of the actuators driving this (MJCF's
    // actuatorfrclimited and actuatorfrcrange).
    void set_actuator_force_limited(bool p_limited);
    bool is_actuator_force_limited() const;
    void set_actuator_force_range(const godot::Vector2 &p_range);
    godot::Vector2 get_actuator_force_range() const;
    void set_limit_margin(double p_value);
    double get_limit_margin() const;
    void set_limit_solref_timeconst(double p_value);
    double get_limit_solref_timeconst() const;
    void set_limit_solref_dampratio(double p_value);
    double get_limit_solref_dampratio() const;
    void set_limit_solimp_dmin(double p_value);
    double get_limit_solimp_dmin() const;
    void set_limit_solimp_dmax(double p_value);
    double get_limit_solimp_dmax() const;
    void set_limit_solimp_width(double p_value);
    double get_limit_solimp_width() const;
    void set_limit_solimp_midpoint(double p_value);
    double get_limit_solimp_midpoint() const;
    void set_limit_solimp_power(double p_value);
    double get_limit_solimp_power() const;

    // Simulation state of a hinge (radians, rad/s) or slide (meters, m/s) joint.
    // Readable whenever the model has compiled; settable only while the game runs.
    double get_joint_position();
    void set_joint_position(double p_position);
    double get_joint_velocity();
    void set_joint_velocity(double p_velocity);

protected:
    static void _bind_methods();
    void _validate_property(godot::PropertyInfo &p_property) const;
    const MujocoAttributeTable *get_attribute_table() const override;
    void overrides_changed() override;

private:
    void changed();
    // Writes the class-able attributes (everything but the frame) into p_joint.
    void write_attributes(mjsJoint *p_joint) const;
    // Index of this joint's value in qpos and qvel. Fails (with an error) for ball
    // and free joints, which have no single value, or if the model hasn't compiled.
    bool get_state_index(int &r_qpos, int &r_dof);

    // Attributes (see mujoco_element.h): type, limited, range (hinge: radians,
    // slide: meters, ball: only y, the maximum angle), damping, stiffness,
    // armature (extra rotor inertia, or mass for slide), frictionloss, ref,
    // springref, and the limit settings limit_margin, limit_solref_* and
    // limit_solimp_* (MuJoCo's margin, solreflimit and solimplimit, by name).
};

VARIANT_ENUM_CAST(MujocoJoint::Type);
