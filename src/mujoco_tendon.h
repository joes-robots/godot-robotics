#pragma once

#include "mujoco_element.h"

#include <vector>

#include <godot_cpp/classes/multi_mesh_instance3d.hpp>
#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/node_path.hpp>
#include <godot_cpp/variant/vector2.hpp>

class MujocoTendonWrap;

// A MuJoCo tendon: a length computed from a path, which can be limited, sprung,
// damped and driven by actuators (a MujocoActuator under it drives it by default).
//
// The path is the tendon's MujocoTendonWrap children, in order:
//   Fixed tendon:   joints, each with a coefficient. The length is the sum of
//                   coefficient * joint position, so it couples joints.
//   Spatial tendon: sites the tendon passes through, sphere or cylinder geoms it
//                   wraps around (between two sites), and pulleys that split it
//                   into branches. It starts and ends at a site, and is drawn
//                   as a tube of `width`.
// The node's own transform is not used.
//
// The spring (stiffness) pulls toward spring_length; -1 means the length in the
// scene pose. Lengths are in meters for spatial tendons, and in the joints' units
// (times the coefficients) for fixed ones.
class MujocoTendon : public MujocoElement {
    GDCLASS(MujocoTendon, MujocoElement);

public:
    mjsBody *add_to_spec(mjSpec *p_spec, mjsBody *p_body, const godot::Transform3D &p_transform, const godot::String &p_name) override;
    const char *get_class_section() const override { return "tendon"; }
    void write_class_defaults(mjsDefault *p_default) override;
    void inherited_changed() override;
    godot::PackedStringArray _get_configuration_warnings() const override;

    void set_limited(bool p_limited);
    bool is_limited() const;
    void set_range(const godot::Vector2 &p_range);
    godot::Vector2 get_range() const;
    void set_stiffness(double p_value);
    double get_stiffness() const;
    void set_damping(double p_value);
    double get_damping() const;
    void set_frictionloss(double p_value);
    double get_frictionloss() const;
    void set_armature(double p_value);
    double get_armature() const;
    void set_spring_length(double p_value);
    double get_spring_length() const;
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
    void set_width(double p_value);
    double get_width() const;
    void set_color(const godot::Color &p_color);
    godot::Color get_color() const;

    // Simulation state: the tendon's length and its rate of change. Readable
    // whenever the model has compiled.
    double get_length();
    double get_velocity();

    // The path, or false with the reason in r_problem (the tendon is then left
    // out of the model). r_fixed tells whether it is a fixed tendon.
    bool get_path(std::vector<MujocoTendonWrap *> &r_wraps, bool &r_fixed, godot::String &r_problem) const;
    // For MujocoTendonWrap: the path changed.
    void path_changed();
    // For MujocoWorld: redraws a spatial tendon's path from the current simulation state.
    void update_drawing();

protected:
    static void _bind_methods();
    void _notification(int p_what);
    void _validate_property(godot::PropertyInfo &p_property) const;
    const MujocoAttributeTable *get_attribute_table() const override;
    void overrides_changed() override;

private:
    void changed();
    // Writes the class-able attributes into p_tendon.
    void write_attributes(mjsTendon *p_tendon) const;
    void update_material();

    // Internal, drawn in the world's frame; created when the tendon enters the tree
    // (class sections never do).
    godot::MultiMeshInstance3D *path_instance = nullptr;

    // Attributes (see mujoco_element.h): limited, range, stiffness, damping,
    // frictionloss, armature, spring_length, the limit settings (as on
    // MujocoJoint), width (radius of the drawn tube) and color.
};

// One step of a MujocoTendon's path; a child of the tendon (see there). Its kind
// follows from `target`: a hinge or slide MujocoJoint (with `coef`), a MujocoSite,
// a sphere or cylinder MujocoGeom to wrap around (optionally on the side of
// `side_site`), or nothing for a pulley (with `divisor`).
class MujocoTendonWrap : public godot::Node {
    GDCLASS(MujocoTendonWrap, godot::Node);

public:
    enum Kind {
        KIND_JOINT,
        KIND_SITE,
        KIND_GEOM,
        KIND_PULLEY,
        KIND_INVALID,
    };

    godot::PackedStringArray _get_configuration_warnings() const override;

    void set_target(const godot::NodePath &p_target);
    godot::NodePath get_target() const;
    void set_coef(double p_coef);
    double get_coef() const;
    void set_side_site(const godot::NodePath &p_site);
    godot::NodePath get_side_site() const;
    void set_divisor(double p_divisor);
    double get_divisor() const;

    // What this step is, with its target (nullptr for a pulley); KIND_INVALID with
    // the reason in r_problem.
    Kind get_kind(godot::Node *&r_target, godot::String &r_problem) const;
    // The side site, or nullptr if none is set; r_problem is set if it is invalid.
    godot::Node *find_side_site(godot::String &r_problem) const;
    // The tendon this belongs to (its parent), or nullptr.
    MujocoTendon *get_tendon() const;

protected:
    static void _bind_methods();
    void _notification(int p_what);
    void _validate_property(godot::PropertyInfo &p_property) const;

private:
    void changed();

    godot::NodePath target;
    double coef = 1.0;
    godot::NodePath side_site;
    double divisor = 1.0;
};

VARIANT_ENUM_CAST(MujocoTendonWrap::Kind);
