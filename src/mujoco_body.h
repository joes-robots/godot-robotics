#pragma once

#include "mujoco_element.h"

// A MuJoCo body (<body> in MJCF).
//
// The node's transform is the body's pose relative to its parent body, exactly
// as pos and quat are in MJCF. Bodies nest: a MujocoBody under another
// MujocoBody is its child body. A body directly under the world (or under plain
// Node3D nodes below it) is a child of MuJoCo's world body.
//
// Rest pose
// ---------
// While the simulation runs, the world moves this node to follow the simulated
// body. But the model is built from the node tree, and it must be built from
// where the body was placed, not from wherever the simulation has carried it.
// So the first time the simulation moves the node, the body remembers the
// transform it had: its rest pose. Compiling uses the rest pose. Moving the
// node yourself (in the editor or from a script) sets a new rest pose.
class MujocoBody : public MujocoElement {
    GDCLASS(MujocoBody, MujocoElement);

public:
    mjsBody *add_to_spec(mjSpec *p_spec, mjsBody *p_body, const godot::Transform3D &p_transform) const override;
    godot::Transform3D get_spec_transform() const override;

    // Moves the node to where the simulation says the body is. Called by the
    // world after stepping. Does not count as an edit, so it doesn't recompile.
    void set_simulated_pose(const godot::Transform3D &p_global_transform);

protected:
    static void _bind_methods();
    void _notification(int p_what);

private:
    // True once the simulation has moved this node away from its rest pose.
    bool simulated = false;
    // The node's transform before the simulation first moved it.
    godot::Transform3D rest_transform;
};
