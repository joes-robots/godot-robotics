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
class MujocoDefault;

class MujocoWorld : public godot::Node3D {
    GDCLASS(MujocoWorld, godot::Node3D);
    // verify() compiles an imported world that isn't in the scene tree.
    friend class MujocoImporter;

public:
    MujocoWorld();

    godot::PackedStringArray _get_configuration_warnings() const override;

    // All physics options at once (the option_* properties), e.g. from an imported model.
    void set_option(const mjOption &p_option);
    const mjOption &get_option() const;

    // Queues one deferred compile; repeated requests in the same frame are merged.
    void request_compile();
    // Runs the queued compile now, if any. Call before reading model/data.
    void run_queued_compile();

    // The current model as MJCF (MuJoCo XML), or an empty string if nothing has compiled.
    godot::String get_mjcf();

    // The MJCF file this world was imported from (res:// inside the project,
    // else an absolute path), for "Reload from Source" in the editor.
    void set_source_file(const godot::String &p_path);
    godot::String get_source_file() const;

    void set_running(bool p_running);
    bool is_running() const;
    void set_max_steps_per_frame(int p_steps);
    int get_max_steps_per_frame() const;
    // Simulated time in seconds since the last compile or reset().
    double get_simulation_time() const;

    // Puts the simulation back in its initial state: time 0, the scene's poses, no motion.
    void reset();
    // Runs p_count MuJoCo steps right away, whether or not `running` is set.
    // Only while the game runs.
    void step(int p_count = 1);

    // For MujocoElements. The current model and data (running a queued compile
    // first), or nullptr if nothing has compiled. Valid until the next compile.
    mjModel *get_model();
    mjData *get_data();
    // Increases with every successful compile, so elements can cache model ids.
    uint64_t get_model_generation() const;
    // Call after writing simulation state (qpos, qvel) outside of stepping:
    // updates the derived quantities and, while the game runs, the body nodes.
    void state_changed();
    // Adds a force and a torque about the body's center of mass, both in this world's
    // MuJoCo frame. They act during every step of the next physics frame, or only the
    // next step when called from a pre_step handler.
    void add_body_wrench(int p_body_id, const mjtNum p_force[3], const mjtNum p_torque[3]);

    // Default classes (MujocoDefault nodes under this world).
    // The class named p_name, or nullptr.
    MujocoDefault *find_default_class(const godot::StringName &p_name) const;
    // The top-level class named "main", or nullptr (then elements without a
    // class use the base defaults).
    MujocoDefault *get_main_class() const;
    godot::PackedStringArray get_default_class_names() const;
    // Call when a class, or which class elements use, changes: recompiles and,
    // once per frame, lets every element refresh what it inherits.
    void default_classes_changed();

    // Conversions between global Godot coordinates and this world's MuJoCo frame
    // (which is the world node's frame, Z-up). Vectors ignore the node's scale.
    void global_to_mujoco_vector(const godot::Vector3 &p_vector, mjtNum r_out[3]) const;
    godot::Vector3 mujoco_to_global_vector(const mjtNum p_vector[3]) const;
    void global_to_mujoco_transform(const godot::Transform3D &p_transform, mjtNum r_pos[3], mjtNum r_quat[4]) const;

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
    // Adds the classes under p_node (the world or a class) to p_spec, with
    // p_parent as their parent class.
    void add_default_classes(godot::Node *p_node, mjSpec *p_spec, mjsDefault *p_parent);
    // Calls inherited_changed() on every element (deferred from default_classes_changed()).
    void refresh_inherited();
    // Advances the simulation by p_delta seconds of game time.
    void step_simulation(double p_delta);
    // One MuJoCo step, preceded by the pre_step signal.
    void step_once();
    // After a frame's steps: clears that frame's forces, brings the derived
    // quantities (poses, velocities) up to date and moves the body nodes.
    void end_frame();
    // Writes the simulated body poses into the MujocoBody nodes.
    void sync_bodies();
    // Copies each MujocoActuator's ctrl into p_data (which mj_makeData and mj_resetData zero).
    void write_controls(const mjModel *p_model, mjData *p_data) const;
    void report_compile_error(const godot::String &p_error);
    void clear_compile_error();

    // Set on NOTIFICATION_READY; edits before that are picked up by the first compile.
    bool initialized = false;
    // A deferred compile is pending (see request_compile()).
    bool compile_queued = false;
    // A deferred refresh_inherited() is pending.
    bool refresh_queued = false;
    // Last compile error, shown as a configuration warning in the editor.
    godot::String compile_error;

    // Physics options, copied into each new spec.
    mjOption option;

    godot::String source_file;

    // Step the simulation while the game runs.
    bool running = true;
    // At most this many MuJoCo steps per physics frame; if the simulation falls
    // further behind, the backlog is dropped (it then runs slower than real time).
    int max_steps_per_frame = 100;
    // Game time not yet simulated, always less than one timestep after a frame.
    double step_accumulator = 0.0;
    // Inside step_once(); mj_step recomputes everything, so state_changed() can skip work.
    bool stepping = false;
    // Emitting pre_step; forces added now act for one step only.
    bool in_pre_step = false;
    // Applied forces (6 per body: force, then torque; MuJoCo's xfrc_applied layout),
    // for the next frame and for the next step.
    std::vector<mjtNum> frame_wrenches;
    std::vector<mjtNum> step_wrenches;
    // Ids of mocap bodies, whose nodes drive the simulation instead of following it.
    std::vector<int> mocap_bodies;
    // See get_model_generation(); 0 means nothing has compiled yet.
    uint64_t model_generation = 0;

    // Bodies collected while building a spec, as (spec body, node instance id).
    std::vector<std::pair<mjsBody *, uint64_t>> building_bodies;
    // Node instance id for each body id of the current model (0 for the world body).
    std::vector<uint64_t> body_nodes;
    // Actuator nodes collected while building a spec (instance ids), then for each
    // actuator id of the current model.
    std::vector<uint64_t> building_actuators;
    std::vector<uint64_t> actuator_nodes;

    // The spec, model and data from the last successful compile.
    std::unique_ptr<mjSpec, MjSpecDeleter> spec;
    std::unique_ptr<mjModel, MjModelDeleter> model;
    std::unique_ptr<mjData, MjDataDeleter> data;
};
