#pragma once

#include <memory>

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
    bool is_compiled() const;
    godot::String get_compile_error() const;

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
    void recompile_spec();
    void handle_failure();

private:
    // True once _ready has made the first compile attempt.
    bool initialized = false;
    // Why the last compile failed; empty if it succeeded.
    godot::String compile_error;

    std::unique_ptr<mjSpec, MjSpecDeleter> spec;
    std::unique_ptr<mjModel, MjModelDeleter> model;
    std::unique_ptr<mjData, MjDataDeleter> data;

    // The option values, in MuJoCo's own struct and conventions. This is where
    // they live: compile() copies them into the spec, and a change is also
    // written to the compiled model straight away.
    mjOption options;
};
