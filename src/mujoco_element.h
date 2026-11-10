#pragma once

#include <godot_cpp/classes/node3d.hpp>
#include <mujoco/mujoco.h>

class MujocoWorld;

// Base class for nodes that become part of a MuJoCo model (bodies, geoms, ...).
// Not instantiable on its own.
//
// An element belongs to its nearest MujocoWorld ancestor; every node in between
// must be a Node3D. Elements hold only Godot-side properties: the world calls
// add_to_spec() on each of them whenever it rebuilds its spec.
class MujocoElement : public godot::Node3D {
    GDCLASS(MujocoElement, godot::Node3D);

public:
    // Adds this element to p_spec.
    //   p_body:      the body this element belongs to (the world body if none).
    //   p_transform: this node's transform relative to p_body, in Godot coordinates.
    //   p_name:      unique MuJoCo name (the node path relative to the world).
    // Returns the body that this node's MuJoCo descendants belong to.
    virtual mjsBody *add_to_spec(mjSpec *p_spec, mjsBody *p_body, const godot::Transform3D &p_transform, const godot::String &p_name);

    // The transform (relative to the parent node) used when building the model.
    virtual godot::Transform3D get_model_transform();

    // Nearest MujocoWorld ancestor, or nullptr.
    MujocoWorld *get_world() const;

    godot::PackedStringArray _get_configuration_warnings() const override;

protected:
    static void _bind_methods() {}
    void _notification(int p_what);

    // Call whenever something that affects the compiled model changes.
    void request_compile() const;

    // This element's id in its world's current model (running a queued compile
    // first), or -1 if it isn't part of a compiled model.
    int get_model_id(mjtObj p_type);

private:
    // Cache for get_model_id(), valid while the world's model generation matches.
    uint64_t cached_generation = 0;
    int cached_id = -1;
};
