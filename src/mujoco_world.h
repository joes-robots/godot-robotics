#pragma once

#include <memory>
#include <vector>

#include "godot_cpp/classes/node3d.hpp"
#include "godot_cpp/variant/utility_functions.hpp"
#include <mujoco/mujoco.h>

struct MjSpecDeleter {
    void operator()(mjSpec* s) const { if (s) mj_deleteSpec(s); }
};
struct MjModelDeleter {
    void operator()(mjModel* m) const { if (m) mj_deleteModel(m); }
};
struct MjDataDeleter {
    void operator()(mjData* d) const { if (d) mj_deleteData(d); }
};

class MujocoWorld : public godot::Node3D {
    GDCLASS(MujocoWorld, godot::Node3D);

public:
    MujocoWorld();

    // Compiling
    bool compile();
    void request_recompile();
    bool is_compiled() const;
    godot::String get_compile_error() const;
    godot::String to_mjcf() const;

    // Simulating
    void step(int p_steps = 1);
    void reset();
    double get_time() const;
    bool has_simulation_error() const;
    void set_paused(bool p_paused);
    bool is_paused() const;
    void _physics_process(double p_delta) override;

    // Signal handlers
    void on_ready();
    void on_entered_tree();
    void on_exited_tree();
    void on_renamed();
    void on_child_order_changed();
    void on_disabled();
    void on_enabled();

protected:
    // Godot built-ins
    static void _bind_methods();
    void _notification(int p_what);

    // Options are dynamic properties, described by the table in mujoco_options.cpp.
    void _get_property_list(godot::List<godot::PropertyInfo> *r_list) const;
    bool _get(const godot::StringName &p_name, godot::Variant &r_value) const;
    bool _set(const godot::StringName &p_name, const godot::Variant &p_value);
    bool _property_can_revert(const godot::StringName &p_name) const;
    bool _property_get_revert(const godot::StringName &p_name, godot::Variant &r_value) const;

    // Mujoco-specific
    void add_children_to_spec(godot::Node *p_node, mjSpec *p_spec, mjsBody *p_body, const godot::Transform3D &p_transform) const;
    void handle_failure(const mjSpec *p_spec);
    void compile_if_pending();
    void find_bodies(godot::Node *p_node);
    bool advance(int p_steps);
    void sync_poses();

private:
    // True once _ready has made the first compile attempt.
    bool initialized = false;
    // True while a deferred compile is queued for the end of this frame.
    bool recompile_pending = false;
    // Why the last compile failed; empty if it succeeded.
    godot::String compile_error;

    // The result of the last successful compile: the spec the model was built
    // from, the model, and its simulation state. All three are replaced together.
    std::unique_ptr<mjSpec, MjSpecDeleter> spec;
    std::unique_ptr<mjModel, MjModelDeleter> model;
    std::unique_ptr<mjData, MjDataDeleter> data;

    // A MujocoBody node and the id of the body it became in the compiled model.
    // The node is held by its instance id, not a pointer, so a node that has
    // been freed is noticed instead of used.
    struct BodyLink {
        uint64_t node_id;
        int body_id;
    };
    // Every body node in the current model, parents before children.
    std::vector<BodyLink> body_links;

    // Automatic stepping in _physics_process is off while this is true.
    bool paused = false;
    // Frame time not yet simulated, carried to the next physics frame.
    double time_accumulator = 0.0;
    // True after MuJoCo reported an error while stepping. Stepping stays off
    // until reset() or a successful compile.
    bool simulation_error = false;

    // The option values, in MuJoCo's own struct and conventions. This is where
    // they live: compile() copies them into the spec, and a change is also
    // written to the compiled model straight away.
    mjOption options;
};
