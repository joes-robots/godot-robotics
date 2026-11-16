#pragma once

#include "mujoco_element.h"

// A MuJoCo body (<body> in MJCF).
//
// The node's transform is the body's pose relative to its parent body, exactly
// as pos and quat are in MJCF. Bodies nest: a MujocoBody under another
// MujocoBody is its child body. A body directly under the world (or under plain
// Node3D nodes below it) is a child of MuJoCo's world body.
//
// Attributes (see MujocoElement for how they are stored):
//
//   mocap   a "motion capture" body: it is not simulated, it goes exactly where
//           it is put, and it pushes other bodies around as it goes. Use it for
//           anything driven from outside the simulation: a hand tracked by a
//           controller, a moving platform, a target. A mocap body must be
//           directly under the world and have no joints.
//
// Rest pose
// ---------
// While the simulation runs, the world moves this node to follow the simulated
// body. But the model is built from the node tree, and it must be built from
// where the body was placed, not from wherever the simulation has carried it.
// So the first time the simulation moves the node, the body remembers the
// transform it had: its rest pose. Compiling uses the rest pose.
//
// Moving the node yourself
// ------------------------
//   - In the editor, or a body that isn't mocap: this places the body. It sets
//     a new rest pose and the model is rebuilt, which restarts the simulation.
//   - A mocap body in a running game: this moves the body in the simulation,
//     with no rebuild. Its rest pose is unchanged, so reset() puts it back.
//
// Simulation state
// ----------------
//   applied_force, applied_torque
//           an external force (N) and torque (N m) on the body, in global
//           coordinates, acting at its centre of mass. They stay in effect
//           until changed; reset() and recompiling clear them.
//   get_linear_velocity(), get_angular_velocity()
//           the body's velocity at its origin and its rate of rotation, in
//           global coordinates.
class MujocoBody : public MujocoElement {
    GDCLASS(MujocoBody, MujocoElement);

public:
    mjsBody *add_to_spec(mjSpec *p_spec, mjsBody *p_body, const godot::Transform3D &p_transform) const override;
    mjtObj get_mujoco_type() const override { return mjOBJ_BODY; }
    godot::Transform3D get_spec_transform() const override;

    // Moves the node to where the simulation says the body is. Called by the
    // world after stepping. Does not count as an edit, so it doesn't recompile.
    void set_simulated_pose(const godot::Transform3D &p_global_transform);

    void set_applied_force(const godot::Vector3 &p_force);
    godot::Vector3 get_applied_force() const;
    void set_applied_torque(const godot::Vector3 &p_torque);
    godot::Vector3 get_applied_torque() const;
    godot::Vector3 get_linear_velocity() const;
    godot::Vector3 get_angular_velocity() const;

protected:
    static void _bind_methods();
    void _get_property_list(godot::List<godot::PropertyInfo> *r_list) const;
    void _validate_property(godot::PropertyInfo &p_property) const;

    const MujocoAttribute *get_attribute_table(int &r_count) const override;
    godot::Variant get_inherited_value(int p_index) const override;
    void transform_changed_by_user() override;

private:
    // True once the simulation has moved this node away from its rest pose.
    bool simulated = false;
    // The node's transform before the simulation first moved it.
    godot::Transform3D rest_transform;

    // The world, if this body is in its compiled model; otherwise nullptr.
    MujocoWorld *get_simulating_world() const;
    // This body's index among the model's mocap bodies, or -1 if it isn't one
    // (or isn't in a compiled model).
    int get_mocap_id() const;

    static godot::Variant read(const mjsBody &p_body, int p_index);
    static void write(mjsBody &r_body, int p_index, const godot::Variant &p_value);
};
