#include "mujoco_world.h"

#include "mujoco_options.h"

using namespace godot;

MujocoWorld::MujocoWorld()
: spec(mj_makeSpec())
{
    mj_defaultOption(&options);

    // Godot initialization
    add_to_group("mujoco", true);
}

// A compile or recompile failed. Record why and report it; the previous model
// and data (if any) are left as they were, so the node keeps working with them.
void MujocoWorld::handle_failure()
{
    const char* error = mjs_getError(spec.get());
    compile_error = error ? String::utf8(error) : String("unknown error");
    ERR_PRINT(String("MuJoCo model '") + String(get_name()) + String("' failed to compile: ") + compile_error);
}

void MujocoWorld::_bind_methods() 
{
    ClassDB::bind_method(D_METHOD("compile"), &MujocoWorld::compile);
    ClassDB::bind_method(D_METHOD("is_compiled"), &MujocoWorld::is_compiled);
    ClassDB::bind_method(D_METHOD("get_compile_error"), &MujocoWorld::get_compile_error);
}

void MujocoWorld::_notification(int p_what)
{
    switch (p_what) {
        case NOTIFICATION_READY:
            on_ready();
            break;
        case NOTIFICATION_ENTER_TREE:
            on_entered_tree();
            break;
        case NOTIFICATION_EXIT_TREE:
            on_exited_tree();
            break;
        case NOTIFICATION_PATH_RENAMED:
            on_renamed();
            break;
        case NOTIFICATION_CHILD_ORDER_CHANGED:
            on_child_order_changed();
            break;
        case NOTIFICATION_DISABLED:
            on_disabled();
            break;
        case NOTIFICATION_ENABLED:
            on_enabled();
            break;
    };
}

// Compiles the spec into a model and its data. Returns true on success.
//
// The first successful call creates the model. Later calls recompile, which
// keeps the simulation state (positions, velocities, time) where it still applies.
// On failure the previous model and data stay in use; see get_compile_error().
bool MujocoWorld::compile()
{
    std::string model_name = String(get_name()).utf8().get_data();
    mjs_setString(spec->modelname, model_name.c_str());
    spec->option = options;

    if (!model) {
        // First compile: nothing to preserve.
        mjModel* new_model = mj_compile(spec.get(), NULL);
        if (!new_model) {
            handle_failure();
            return false;
        }
        model.reset(new_model);
        data.reset(mj_makeData(new_model));
    } else {
        // mj_recompile deletes the model and data it was given when it fails, so
        // it works on copies: the originals are only replaced after a success.
        mjModel* new_model = mj_copyModel(NULL, model.get());
        mjData* new_data = mj_copyData(NULL, model.get(), data.get());
        if (mj_recompile(spec.get(), NULL, new_model, new_data) != 0) {
            handle_failure();
            return false;
        }
        // data is replaced first: it must be deleted before the model it belongs to.
        data.reset(new_data);
        model.reset(new_model);
    }

    compile_error = String();
    notify_property_list_changed();
    return true;
}

// Recompiles after a change, once the node has been through its first compile
// attempt in _ready. Before that, changes just accumulate in the spec.
void MujocoWorld::recompile_spec()
{
    if (!initialized) {
        return;
    }
    compile();
}

bool MujocoWorld::is_compiled() const
{
    return model != nullptr;
}

String MujocoWorld::get_compile_error() const
{
    return compile_error;
}

void MujocoWorld::on_ready() {
    // called last when all child nodes are added to the scene tree
    initialized = true;
    compile();
    UtilityFunctions::print_verbose("READY: " + get_name());
}

void MujocoWorld::on_entered_tree() 
{
    // called first, then child nodes get their _enter_tree callbacks triggered after
    UtilityFunctions::print_verbose("ENTERED TREE: " + get_name());
}

void MujocoWorld::on_exited_tree() 
{
    UtilityFunctions::print_verbose("EXITED TREE: " + get_name());
}

void MujocoWorld::on_renamed()
{
    // compile() copies the node name into the spec.
    recompile_spec();
    UtilityFunctions::print_verbose("RENAMED: " + get_name());
}

void MujocoWorld::on_child_order_changed() 
{
    recompile_spec();
    UtilityFunctions::print_verbose("CHILD ORDER CHANGED: " + get_name());
}

void MujocoWorld::on_disabled() 
{
    UtilityFunctions::print_verbose("DISABLED: " + get_name());
}

void MujocoWorld::on_enabled() 
{
    recompile_spec();
    UtilityFunctions::print_verbose("ENABLED: " + get_name());
}

// Options: dynamic properties, forwarded to the table in mujoco_options.cpp.

void MujocoWorld::_get_property_list(List<PropertyInfo> *r_list) const
{
    mujoco_options::get_property_list(r_list);
}

bool MujocoWorld::_get(const StringName &p_name, Variant &r_value) const
{
    return mujoco_options::get(options, p_name, r_value);
}

bool MujocoWorld::_set(const StringName &p_name, const Variant &p_value)
{
    if (!mujoco_options::set(options, p_name, p_value)) {
        return false;
    }
    // Options don't change the model's structure, so there is nothing to
    // recompile: the running model takes the new values directly.
    if (model) {
        model->opt = options;
    }
    return true;
}

// These two tell Godot what each option's default is. With them, the inspector
// shows a revert arrow on changed options, and scenes only store changed ones.
bool MujocoWorld::_property_can_revert(const StringName &p_name) const
{
    Variant default_value;
    return mujoco_options::get_default(p_name, default_value);
}

bool MujocoWorld::_property_get_revert(const StringName &p_name, Variant &r_value) const
{
    return mujoco_options::get_default(p_name, r_value);
}
