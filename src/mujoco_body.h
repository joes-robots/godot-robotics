#pragma once

#include "mujoco_element.h"

// A MuJoCo body: a rigid frame that its child geoms and joints attach to.
// Mass and inertia are computed by MuJoCo from the body's geoms.
// Without a joint the body is welded to its parent body.
//
// While the game runs, the world writes the simulated pose into this node
// every physics frame.
class MujocoBody : public MujocoElement {
    GDCLASS(MujocoBody, MujocoElement);

public:
    mjsBody *add_to_spec(mjSpec *p_spec, mjsBody *p_body, const godot::Transform3D &p_transform, const godot::String &p_name) override;

    // In the editor, the node's transform. At runtime, the transform it had when
    // first compiled: the simulation moves the node afterwards, and a recompile
    // must not take a simulated pose as the body's reference pose.
    godot::Transform3D get_model_transform() override;

protected:
    static void _bind_methods() {}
    void _notification(int p_what);

private:
    bool has_rest_transform = false;
    godot::Transform3D rest_transform;
};
