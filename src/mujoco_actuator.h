#pragma once

#include "mujoco_element.h"

#include <godot_cpp/variant/node_path.hpp>
#include <godot_cpp/variant/vector2.hpp>

class MujocoJoint;

// A MuJoCo actuator driving a hinge or slide joint, or a tendon: `target`, by
// default its parent (".."). The node's transform is not used.
//
// The control input `ctrl` means, by type:
//   Motor:    force or torque (before gear).
//   Position: target joint position, like MujocoJoint.get_joint_position(), or
//             target tendon length, like MujocoTendon.get_length().
//   Velocity: target joint velocity, or rate of change of the tendon length.
//
// (Before tendons, `target` was called `joint`; scenes that saved it still load.)
// The force it produces is gear * actuator force, with the actuator force
// gain * ctrl + bias (see get_force()).
//
// `ctrl` is the node's own value, not simulation state: it survives recompiles and
// reset(), and changing it never recompiles. All other properties change the model.
class MujocoActuator : public MujocoElement {
    GDCLASS(MujocoActuator, MujocoElement);

public:
    enum Type {
        TYPE_MOTOR,
        TYPE_POSITION,
        TYPE_VELOCITY,
    };

    mjsBody *add_to_spec(mjSpec *p_spec, mjsBody *p_body, const godot::Transform3D &p_transform, const godot::String &p_name) override;
    const char *get_class_section() const override { return "actuator"; }
    void write_class_defaults(mjsDefault *p_default) override;
    godot::PackedStringArray _get_configuration_warnings() const override;

    void set_type(Type p_type);
    Type get_type() const;
    void set_target(const godot::NodePath &p_target);
    godot::NodePath get_target() const;
    void set_ctrl(double p_ctrl);
    double get_ctrl() const;
    void set_gear(double p_gear);
    double get_gear() const;
    void set_kp(double p_kp);
    double get_kp() const;
    void set_kv(double p_kv);
    double get_kv() const;
    void set_ctrl_limited(bool p_limited);
    bool is_ctrl_limited() const;
    void set_ctrl_range(const godot::Vector2 &p_range);
    godot::Vector2 get_ctrl_range() const;
    void set_force_limited(bool p_limited);
    bool is_force_limited() const;
    void set_force_range(const godot::Vector2 &p_range);
    godot::Vector2 get_force_range() const;

    // The actuator force from the last step, after force limits and before gear
    // (the joint receives gear times this). Errors if the actuator isn't compiled.
    double get_force();

    // For MujocoWorld: this actuator's id in the current model, or -1.
    int get_actuator_id();
    // The joint or tendon this actuator drives, or nullptr (with the reason in
    // r_problem) if it has none and is left out of the model.
    MujocoElement *find_target(godot::String &r_problem) const;

protected:
    static void _bind_methods();
    void _validate_property(godot::PropertyInfo &p_property) const;
    const MujocoAttributeTable *get_attribute_table() const override;
    void overrides_changed() override;
    // Loads the old `joint` property as `target`.
    bool _set(const godot::StringName &p_name, const godot::Variant &p_value);
    bool _get(const godot::StringName &p_name, godot::Variant &r_ret) const;

private:
    void changed();
    // Writes gear and the control and force ranges into p_actuator. (Gains depend
    // on the type, which is the actuator's own; classes don't write them.)
    void write_ranges(mjsActuator *p_actuator) const;
    // Writes ctrl into the simulation, if this actuator is part of a compiled model.
    void write_ctrl();

    Type type = TYPE_MOTOR;
    godot::NodePath target = godot::NodePath("..");
    double ctrl = 0.0;
    // Attributes (see mujoco_element.h): gear, kp (position: force per unit of
    // position error), kv (position: damping; velocity: force per unit of velocity
    // error), ctrl_limited, ctrl_range, force_limited, force_range.
};

VARIANT_ENUM_CAST(MujocoActuator::Type);
