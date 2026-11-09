#pragma once

#include <memory>

#include <godot_cpp/classes/node.hpp>
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

class MujocoSpec : public godot::Node {
    GDCLASS(MujocoSpec, godot::Node);

public:
    MujocoSpec();

    godot::PackedStringArray _get_configuration_warnings() const override;

protected:
    // Godot built-ins
    static void _bind_methods();
    void _notification(int p_what);

    // Dynamic properties: the "option_*" fields of spec->option (see mujoco_options.h)
    bool _set(const godot::StringName &p_name, const godot::Variant &p_value);
    bool _get(const godot::StringName &p_name, godot::Variant &r_ret) const;
    void _get_property_list(godot::List<godot::PropertyInfo> *p_list) const;
    bool _property_can_revert(const godot::StringName &p_name) const;
    bool _property_get_revert(const godot::StringName &p_name, godot::Variant &r_property) const;

    // Mujoco-specific
    // Compiles immediately. Prefer request_compile() for anything triggered by edits.
    void compile_spec();
    // Queues one deferred compile; repeated requests in the same frame are merged.
    void request_compile();
    // Runs the queued compile now, if any. Call before reading model/data.
    void flush_compile();
    void report_compile_error(const godot::String &p_error);
    void clear_compile_error();

private:
    // Notification handlers, dispatched from _notification()
    void on_ready();
    void on_renamed();
    void on_child_order_changed();

    // Set on NOTIFICATION_READY; property changes before that only edit the spec.
    bool initialized = false;
    // A deferred compile is pending (see request_compile()).
    bool compile_queued = false;
    // Last compile error, shown as a configuration warning in the editor.
    godot::String compile_error;

    std::unique_ptr<mjSpec, MjSpecDeleter> spec;
    std::unique_ptr<mjModel, MjModelDeleter> model;
    std::unique_ptr<mjData, MjDataDeleter> data;
};
