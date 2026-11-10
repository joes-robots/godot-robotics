#pragma once

#include "mujoco_element.h"

#include <godot_cpp/variant/vector2.hpp>

// A MuJoCo joint: gives the body it belongs to (the nearest MujocoBody
// ancestor) degrees of freedom relative to that body's parent.
//
// The node's position is the joint anchor and its local Y axis is the hinge or
// slide axis (MuJoCo's default joint axis, local z, converted to Godot). Free
// joints ignore both and must belong to a body whose parent is the world.
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

protected:
    static void _bind_methods();
    void _validate_property(godot::PropertyInfo &p_property) const;

private:
    void changed();

    Type type = TYPE_HINGE;
    bool limited = false;
    // Hinge: degrees. Slide: meters. Ball: only y is used (maximum angle in degrees).
    godot::Vector2 range{ -90, 90 };
    double damping = 0.0;      // force (or torque) per unit velocity
    double stiffness = 0.0;    // spring force (or torque) per unit displacement
    double armature = 0.0;     // extra rotor inertia (or mass for slide)
    double frictionloss = 0.0; // dry friction force (or torque)
};

VARIANT_ENUM_CAST(MujocoJoint::Type);
