#pragma once

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include <godot_cpp/classes/node3d.hpp>
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

// Root of a MuJoCo simulation. Its transform is the MuJoCo world frame.
//
// The node tree is the source of truth: every compile builds a fresh mjSpec
// from this node's properties and its MujocoElement descendants, compiles it,
// and replaces the model and data. A structural change therefore resets the
// simulation state.
//
// While the game runs (never in the editor), the world steps the simulation in
// _physics_process and writes each body's simulated pose into its MujocoBody.
// Option changes at runtime are applied to the running model without a reset.
class MujocoWorld : public godot::Node3D {
    GDCLASS(MujocoWorld, godot::Node3D);

public:
    MujocoWorld();

    godot::PackedStringArray _get_configuration_warnings() const override;

    // Queues one deferred compile; repeated requests in the same frame are merged.
    void request_compile();
    // Runs the queued compile now, if any. Call before reading model/data.
    void flush_compile();

    // The current model as MJCF (MuJoCo XML), or an empty string if nothing has compiled.
    godot::String get_mjcf();

    void set_running(bool p_running);
    bool is_running() const;
    void set_max_steps_per_frame(int p_steps);
    int get_max_steps_per_frame() const;
    // Simulated time in seconds since the last compile (which resets the simulation).
    double get_simulation_time() const;

protected:
    // Godot built-ins
    static void _bind_methods();
    void _notification(int p_what);

    // Dynamic properties: the "option_*" fields of `option` (see mujoco_options.h)
    bool _set(const godot::StringName &p_name, const godot::Variant &p_value);
    bool _get(const godot::StringName &p_name, godot::Variant &r_ret) const;
    void _get_property_list(godot::List<godot::PropertyInfo> *p_list) const;
    bool _property_can_revert(const godot::StringName &p_name) const;
    bool _property_get_revert(const godot::StringName &p_name, godot::Variant &r_property) const;

private:
    // Builds a new spec from the node tree and compiles it. On success the new
    // spec, model and data replace the current ones; on failure the last good
    // ones are kept and the error is reported.
    void compile();
    // Adds the MujocoElements under p_node to p_spec. p_body is the body they belong to and
    // p_to_body the transform from p_node's frame to p_body's frame (Godot coordinates).
    void add_subtree(godot::Node *p_node, mjSpec *p_spec, mjsBody *p_body, const godot::Transform3D &p_to_body);
    // Advances the simulation by p_delta seconds of game time.
    void step_simulation(double p_delta);
    // Writes the simulated body poses into the MujocoBody nodes.
    void sync_bodies();
    void report_compile_error(const godot::String &p_error);
    void clear_compile_error();

    // Set on NOTIFICATION_READY; edits before that are picked up by the first compile.
    bool initialized = false;
    // A deferred compile is pending (see request_compile()).
    bool compile_queued = false;
    // Last compile error, shown as a configuration warning in the editor.
    godot::String compile_error;

    // Physics options, copied into each new spec.
    mjOption option;

    // Step the simulation while the game runs.
    bool running = true;
    // At most this many MuJoCo steps per physics frame; if the simulation falls
    // further behind, the backlog is dropped (it then runs slower than real time).
    int max_steps_per_frame = 100;
    // Game time not yet simulated, always less than one timestep after a frame.
    double step_accumulator = 0.0;

    // Bodies collected while building a spec, as (spec body, node instance id).
    std::vector<std::pair<mjsBody *, uint64_t>> building_bodies;
    // Node instance id for each body id of the current model (0 for the world body).
    std::vector<uint64_t> body_nodes;

    // The spec, model and data from the last successful compile.
    std::unique_ptr<mjSpec, MjSpecDeleter> spec;
    std::unique_ptr<mjModel, MjModelDeleter> model;
    std::unique_ptr<mjData, MjDataDeleter> data;
};
