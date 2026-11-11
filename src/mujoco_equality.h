#pragma once

#include "mujoco_element.h"

#include <godot_cpp/variant/node_path.hpp>

// A MuJoCo equality constraint: keeps two things in a fixed relation.
//
//   Connect: object1 and object2 are bodies (object2 empty: the world). They stay
//            joined at a point, like a ball joint: the node's position, a point
//            fixed in object1's body. With two sites instead, the sites stay together.
//   Weld:    object1 and object2 are bodies (object2 empty: the world) or two sites.
//            They keep the relative pose they have in the scene. torque_scale
//            weighs rotation against translation.
//   Joint:   object1 and object2 are hinge or slide joints. Joint 1's position
//            follows a polynomial of joint 2's:
//                q1 = poly0 + poly1 q2 + poly2 q2^2 + poly3 q2^3 + poly4 q2^4
//            measured from each joint's ref. With object2 empty, q1 = poly0.
//   Tendon:  like Joint, with the lengths of two tendons.
//
// object1 is the parent ("..") by default, so the node usually sits under the
// first body, joint or tendon. Only Connect uses the node's transform.
//
// `active` can be switched while the game runs (e.g. a weld to grasp something);
// like an actuator's ctrl it never recompiles, and it survives reset().
class MujocoEquality : public MujocoElement {
    GDCLASS(MujocoEquality, MujocoElement);

public:
    // Values are MuJoCo's mjtEq.
    enum Type {
        TYPE_CONNECT = mjEQ_CONNECT,
        TYPE_WELD = mjEQ_WELD,
        TYPE_JOINT = mjEQ_JOINT,
        TYPE_TENDON = mjEQ_TENDON,
    };

    mjsBody *add_to_spec(mjSpec *p_spec, mjsBody *p_body, const godot::Transform3D &p_transform, const godot::String &p_name) override;
    const char *get_class_section() const override { return "equality"; }
    void write_class_defaults(mjsDefault *p_default) override;
    godot::PackedStringArray _get_configuration_warnings() const override;

    void set_type(Type p_type);
    Type get_type() const;
    void set_object1(const godot::NodePath &p_path);
    godot::NodePath get_object1() const;
    void set_object2(const godot::NodePath &p_path);
    godot::NodePath get_object2() const;
    void set_active(bool p_active);
    bool is_active() const;
    void set_torque_scale(double p_scale);
    double get_torque_scale() const;
    void set_poly0(double p_value);
    double get_poly0() const;
    void set_poly1(double p_value);
    double get_poly1() const;
    void set_poly2(double p_value);
    double get_poly2() const;
    void set_poly3(double p_value);
    double get_poly3() const;
    void set_poly4(double p_value);
    double get_poly4() const;
    // Solver settings, named like MujocoGeom's (MuJoCo's solref and solimp).
    void set_solref_timeconst(double p_value);
    double get_solref_timeconst() const;
    void set_solref_dampratio(double p_value);
    double get_solref_dampratio() const;
    void set_solimp_dmin(double p_value);
    double get_solimp_dmin() const;
    void set_solimp_dmax(double p_value);
    double get_solimp_dmax() const;
    void set_solimp_width(double p_value);
    double get_solimp_width() const;
    void set_solimp_midpoint(double p_value);
    double get_solimp_midpoint() const;
    void set_solimp_power(double p_value);
    double get_solimp_power() const;

    // The MuJoCo names of the two objects and their type, or false with the reason
    // in r_problem (the constraint is then left out of the model). r_name2 is empty
    // for "none" (Joint, Tendon) and "world" for the world.
    bool resolve(godot::String &r_name1, godot::String &r_name2, mjtObj &r_type, godot::String &r_problem) const;

protected:
    static void _bind_methods();
    void _validate_property(godot::PropertyInfo &p_property) const;
    const MujocoAttributeTable *get_attribute_table() const override;
    void overrides_changed() override;

private:
    void changed();
    void write_attributes(mjsEquality *p_equality) const;
    // Writes `active` into the compiled model and simulation, if any.
    void write_active();
    // The node p_path points to, as one of p_type's objects: its MuJoCo name and
    // type, or false with the reason. p_optional: an empty path is allowed.
    bool resolve_object(const godot::NodePath &p_path, const char *p_property, bool p_optional, godot::String &r_name, mjtObj &r_type,
            godot::String &r_problem) const;

    Type type = TYPE_CONNECT;
    godot::NodePath object1 = godot::NodePath("..");
    godot::NodePath object2;
    bool active = true;
    double torque_scale = 1.0;
    double poly[5] = { 0.0, 1.0, 0.0, 0.0, 0.0 };
    // Attributes (see mujoco_element.h): solref_* and solimp_*.
};

VARIANT_ENUM_CAST(MujocoEquality::Type);
