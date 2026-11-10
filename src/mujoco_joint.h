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
// Joint positions are measured from the pose in the scene, which is position 0.
// Positive hinge rotation follows the right-hand rule about the axis.
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

    // Simulation state of a hinge (radians, rad/s) or slide (meters, m/s) joint.
    // Readable whenever the model has compiled; settable only while the game runs.
    double get_joint_position();
    void set_joint_position(double p_position);
    double get_joint_velocity();
    void set_joint_velocity(double p_velocity);

protected:
    static void _bind_methods();
    void _validate_property(godot::PropertyInfo &p_property) const;

private:
    void changed();
    // Index of this joint's value in qpos and qvel. Fails (with an error) for ball
    // and free joints, which have no single value, or if the model hasn't compiled.
    bool get_state_index(int &r_qpos, int &r_dof);

    Type type = TYPE_HINGE;
    bool limited = false;
    // Hinge: radians. Slide: meters. Ball: only y is used (maximum angle in radians).
    godot::Vector2 range = godot::Vector2(-godot::Math::PI / 2, godot::Math::PI / 2);
    double damping = 0.0;      // force (or torque) per unit velocity
    double stiffness = 0.0;    // spring force (or torque) per unit displacement
    double armature = 0.0;     // extra rotor inertia (or mass for slide)
    double frictionloss = 0.0; // dry friction force (or torque)
};

VARIANT_ENUM_CAST(MujocoJoint::Type);
