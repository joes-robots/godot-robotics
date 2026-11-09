#include "mujoco_spec.h"
#include "mujoco_options.h"

#include <godot_cpp/variant/callable_method_pointer.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

MujocoSpec::MujocoSpec()
: spec(mj_makeSpec())
{
    // Non-persistent: not written into saved scenes.
    add_to_group("mujoco");
}

// Record the error and surface it in the editor. The node stays in the tree so
// the user can fix the offending property; the next successful compile clears it.
void MujocoSpec::report_compile_error(const String &p_error)
{
    compile_error = p_error;
    ERR_PRINT(vformat("MuJoCo model '%s' failed to compile: %s", get_name(), p_error));
    update_configuration_warnings();
}

void MujocoSpec::clear_compile_error()
{
    if (compile_error.is_empty()) {
        return;
    }
    compile_error = String();
    update_configuration_warnings();
}

PackedStringArray MujocoSpec::_get_configuration_warnings() const
{
    PackedStringArray warnings = Node::_get_configuration_warnings();
    if (!compile_error.is_empty()) {
        warnings.push_back("MuJoCo model failed to compile:\n" + compile_error);
    }
    return warnings;
}

void MujocoSpec::_bind_methods()
{
    // Properties are dynamic (see _get_property_list); only constants are bound here.
    mujoco_options::bind_constants(get_class_static());
}

bool MujocoSpec::_set(const StringName &p_name, const Variant &p_value)
{
    bool changed = false;
    if (!mujoco_options::set(spec->option, p_name, p_value, changed)) {
        return false;
    }
    if (changed) {
        request_compile();
    }
    return true;
}

bool MujocoSpec::_get(const StringName &p_name, Variant &r_ret) const
{
    return mujoco_options::get(spec->option, p_name, r_ret);
}

void MujocoSpec::_get_property_list(List<PropertyInfo> *p_list) const
{
    mujoco_options::get_property_list(p_list);
}

bool MujocoSpec::_property_can_revert(const StringName &p_name) const
{
    Variant unused;
    return mujoco_options::get_default(p_name, unused);
}

bool MujocoSpec::_property_get_revert(const StringName &p_name, Variant &r_property) const
{
    return mujoco_options::get_default(p_name, r_property);
}

void MujocoSpec::_notification(int p_what)
{
    switch (p_what) {
        case NOTIFICATION_READY:
            on_ready();
            break;
        case NOTIFICATION_PATH_RENAMED:
            on_renamed();
            break;
        case NOTIFICATION_CHILD_ORDER_CHANGED:
            on_child_order_changed();
            break;
    };
}

void MujocoSpec::compile_spec()
{
    if (!spec) {
        return;
    }

    const CharString model_name = String(get_name()).utf8();
    mjs_setString(spec->modelname, model_name.get_data());

    if (model && data) {
        // Recompile in place, preserving simulation state. On failure MuJoCo has
        // already freed BOTH model and data, so drop ownership without deleting.
        if (mj_recompile(spec.get(), nullptr, model.get(), data.get()) != 0) {
            (void)model.release();
            (void)data.release();
            report_compile_error(mjs_getError(spec.get()));
            return;
        }
    } else {
        // First compile, or the previous compile failed: build from scratch.
        data.reset();
        model.reset();

        std::unique_ptr<mjModel, MjModelDeleter> new_model(mj_compile(spec.get(), nullptr));
        if (!new_model) {
            report_compile_error(mjs_getError(spec.get()));
            return;
        }
        std::unique_ptr<mjData, MjDataDeleter> new_data(mj_makeData(new_model.get()));
        if (!new_data) {
            report_compile_error("mj_makeData failed to allocate simulation data.");
            return;
        }
        model = std::move(new_model);
        data = std::move(new_data);
    }

    clear_compile_error();
    UtilityFunctions::print_verbose(vformat("MuJoCo: compiled model '%s'.", get_name()));
}

void MujocoSpec::request_compile()
{
    // Before ready, edits only change the spec; on_ready() does the first compile.
    if (!initialized || compile_queued) {
        return;
    }
    compile_queued = true;
    callable_mp(this, &MujocoSpec::flush_compile).call_deferred();
}

void MujocoSpec::flush_compile()
{
    if (!compile_queued) {
        return;
    }
    compile_queued = false;
    compile_spec();
}

void MujocoSpec::on_ready() {
    // called last when all child nodes are added to the scene tree
    initialized = true;
    compile_queued = false;
    compile_spec();
}

void MujocoSpec::on_renamed()
{
    // compile_spec() picks up the new name
    request_compile();
}

void MujocoSpec::on_child_order_changed() 
{
    request_compile();
}
