#pragma once

#include "mujoco_element.h"

#include <godot_cpp/variant/node_path.hpp>
#include <godot_cpp/variant/vector2.hpp>

class MujocoJoint;

// A MuJoCo actuator driving a hinge or slide joint. The node's transform is not
// used; by default it drives its parent joint (`joint` is "..").
//
// The control input `ctrl` means, by type:
//   Motor:    force or torque (before gear).
//   Position: target joint position, like MujocoJoint.get_joint_position().
//   Velocity: target joint velocity.
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
    godot::PackedStringArray _get_configuration_warnings() const override;

    void set_type(Type p_type);
    Type get_type() const;
    void set_joint(const godot::NodePath &p_joint);
    godot::NodePath get_joint() const;
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
    // The joint this actuator drives, or nullptr (with the reason in r_problem) if
    // it has none and is left out of the model.
    MujocoJoint *find_joint(godot::String &r_problem) const;

protected:
    static void _bind_methods();
    void _validate_property(godot::PropertyInfo &p_property) const;

private:
    void changed();
    // Writes ctrl into the simulation, if this actuator is part of a compiled model.
    void write_ctrl();

    Type type = TYPE_MOTOR;
    godot::NodePath joint = godot::NodePath("..");
    double ctrl = 0.0;
    double gear = 1.0;
    double kp = 1.0; // position: force per unit of position error
    double kv = 0.0; // position: damping; velocity: force per unit of velocity error
    bool ctrl_limited = false;
    godot::Vector2 ctrl_range = godot::Vector2(-1, 1);
    bool force_limited = false;
    godot::Vector2 force_range = godot::Vector2(-1, 1);
};

VARIANT_ENUM_CAST(MujocoActuator::Type);
