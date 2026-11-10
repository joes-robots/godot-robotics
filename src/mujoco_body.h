#pragma once

#include "mujoco_element.h"

// A MuJoCo body: a rigid frame that its child geoms and joints attach to.
// Mass and inertia are computed by MuJoCo from the body's geoms.
// Without a joint the body is welded to its parent body.
//
// While the game runs, the world writes the simulated pose into this node
// every physics frame, except for mocap bodies: those are moved by scripts
// (like kinematic bodies) and the simulation follows the node.
class MujocoBody : public MujocoElement {
    GDCLASS(MujocoBody, MujocoElement);

public:
    mjsBody *add_to_spec(mjSpec *p_spec, mjsBody *p_body, const godot::Transform3D &p_transform, const godot::String &p_name) override;

    // In the editor, the node's transform. At runtime, the transform it had when
    // first compiled: the simulation moves the node afterwards, and a recompile
    // must not take a simulated pose as the body's reference pose.
    godot::Transform3D get_model_transform() override;

    godot::PackedStringArray _get_configuration_warnings() const override;

    void set_mocap(bool p_mocap);
    bool is_mocap() const;

    // Velocity of the body's origin, in global coordinates (zero for mocap bodies).
    godot::Vector3 get_linear_velocity();
    godot::Vector3 get_angular_velocity();

    // For bodies whose only joint is a free joint, while the game runs.
    void set_linear_velocity(const godot::Vector3 &p_velocity);
    void set_angular_velocity(const godot::Vector3 &p_velocity);
    // Moves the body to p_transform (global) without changing its velocity.
    void teleport(const godot::Transform3D &p_transform);

    // Forces and torques in global coordinates, like RigidBody3D's. They act during
    // the next physics frame, or only the next MuJoCo step when called from pre_step.
    // p_position is the offset from the body's origin, in global orientation.
    void apply_force(const godot::Vector3 &p_force, const godot::Vector3 &p_position = godot::Vector3());
    void apply_central_force(const godot::Vector3 &p_force);
    void apply_torque(const godot::Vector3 &p_torque);

protected:
    static void _bind_methods();
    void _notification(int p_what);

private:
    // This body's id in the compiled model, or -1 (with an error naming p_action).
    int get_body_id(const char *p_action);
    // Index of the free joint's values in qpos and qvel; fails with an error if the
    // body's only joint isn't a free joint, or outside the running game.
    bool get_free_joint_index(int &r_qpos, int &r_dof);
    void add_wrench(const godot::Vector3 &p_force, const godot::Vector3 *p_position, const godot::Vector3 &p_torque);

    bool mocap = false;
    bool has_rest_transform = false;
    godot::Transform3D rest_transform;
};
