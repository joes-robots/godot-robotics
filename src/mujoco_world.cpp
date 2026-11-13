#include "mujoco_world.h"

#include <string>

#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/core/object.hpp>

#include "mujoco_body.h"
#include "mujoco_convert.h"
#include "mujoco_element.h"
#include "mujoco_options.h"
#include "register_types.h"

using namespace godot;

MujocoWorld::MujocoWorld()
{
    mj_defaultOption(&options);

    // Godot initialization
    add_to_group("mujoco", true);
}

// A compile failed. Record why and report it; the previous spec, model and data
// (if any) are left as they were, so the node keeps working with them.
void MujocoWorld::handle_failure(const mjSpec *p_spec)
{
    // mjs_getError takes a non-const spec but only reads from it.
    const char* error = mjs_getError(const_cast<mjSpec *>(p_spec));
    compile_error = error ? String::utf8(error) : String("unknown error");
    ERR_PRINT(String("MuJoCo model '") + String(get_name()) + String("' failed to compile: ") + compile_error);
}

void MujocoWorld::_bind_methods() 
{
    ClassDB::bind_method(D_METHOD("compile"), &MujocoWorld::compile);
    ClassDB::bind_method(D_METHOD("request_recompile"), &MujocoWorld::request_recompile);
    ClassDB::bind_method(D_METHOD("to_mjcf"), &MujocoWorld::to_mjcf);

    ClassDB::bind_method(D_METHOD("step", "steps"), &MujocoWorld::step, DEFVAL(1));
    ClassDB::bind_method(D_METHOD("reset"), &MujocoWorld::reset);
    ClassDB::bind_method(D_METHOD("get_time"), &MujocoWorld::get_time);
    ClassDB::bind_method(D_METHOD("has_simulation_error"), &MujocoWorld::has_simulation_error);
    ClassDB::bind_method(D_METHOD("set_paused", "paused"), &MujocoWorld::set_paused);
    ClassDB::bind_method(D_METHOD("is_paused"), &MujocoWorld::is_paused);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "paused"), "set_paused", "is_paused");
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

// Builds a new model from the node tree. Returns true on success.
//
// The node tree is the source of truth: every compile starts from an empty spec
// and adds each MuJoCo node below this world to it. Nothing is carried over from
// the previous model, so the simulation restarts from its initial state.
//
// On failure the previous spec, model and data stay in use; see get_compile_error().
bool MujocoWorld::compile()
{
    recompile_pending = false;

    std::unique_ptr<mjSpec, MjSpecDeleter> new_spec(mj_makeSpec());
    mjs_setString(new_spec->modelname, String(get_name()).utf8().get_data());
    new_spec->option = options;

    // Every spec starts with one body, called "world": the fixed root that
    // everything else hangs off.
    mjsBody *world_body = mjs_findBody(new_spec.get(), "world");
    add_children_to_spec(this, new_spec.get(), world_body, Transform3D());

    mjModel *new_model = mj_compile(new_spec.get(), NULL);
    if (!new_model) {
        handle_failure(new_spec.get());
        return false;
    }

    // The old data is deleted before the model it belongs to.
    data.reset(mj_makeData(new_model));
    model.reset(new_model);
    spec = std::move(new_spec);

    // Match each body node to its body in the new model.
    body_links.clear();
    find_bodies(this);

    // A new model starts from its initial state, with a clean slate.
    time_accumulator = 0.0;
    simulation_error = false;
    mujoco_take_engine_error();
    // mj_forward works out everything that follows from the state (body poses
    // among them) without advancing time.
    mj_forward(model.get(), data.get());
    // Put the nodes where the new model starts (a no-op in the editor).
    sync_poses();

    compile_error = String();
    notify_property_list_changed();
    return true;
}

// Adds the MuJoCo elements below p_node to the spec, depth first.
//
//   p_body       the MuJoCo body that elements found here belong to.
//   p_transform  the pose of p_node relative to p_body.
void MujocoWorld::add_children_to_spec(Node *p_node, mjSpec *p_spec, mjsBody *p_body, const Transform3D &p_transform) const
{
    for (int i = 0; i < p_node->get_child_count(); i++) {
        Node *child = p_node->get_child(i);

        // A world inside a world is its own, separate model.
        if (Object::cast_to<MujocoWorld>(child)) {
            continue;
        }

        // The child's pose relative to p_body. Nodes that aren't Node3D have no
        // transform of their own and pass their parent's through.
        Transform3D transform = p_transform;
        if (const Node3D *node_3d = Object::cast_to<Node3D>(child)) {
            // A body that the simulation has moved reports where it was placed,
            // not where it is now; see MujocoBody.
            const MujocoElement *as_element = Object::cast_to<MujocoElement>(child);
            Transform3D local = as_element ? as_element->get_spec_transform() : node_3d->get_transform();
            // MuJoCo has no scale. Warn, and use the transform without it.
            if (!local.basis.get_scale().is_equal_approx(Vector3(1, 1, 1))) {
                WARN_PRINT(String("MuJoCo ignores the scale of '") + String(get_path_to(child)) + String("'."));
                local.basis = local.basis.orthonormalized();
            }
            transform = p_transform * local;
        }

        if (const MujocoElement *element = Object::cast_to<MujocoElement>(child)) {
            // An element adds itself, and says which body its children belong to.
            // Inside a new body, poses are relative to that body, so the
            // accumulated transform starts again from identity.
            if (mjsBody *child_body = element->add_to_spec(p_spec, p_body, transform)) {
                add_children_to_spec(child, p_spec, child_body, Transform3D());
            }
        } else {
            // Any other node is just grouping: look through it.
            add_children_to_spec(child, p_spec, p_body, transform);
        }
    }
}

// Records every MujocoBody below p_node with its id in the compiled model.
// Names in the model are node paths, so the lookup is by name.
void MujocoWorld::find_bodies(Node *p_node)
{
    for (int i = 0; i < p_node->get_child_count(); i++) {
        Node *child = p_node->get_child(i);
        if (Object::cast_to<MujocoWorld>(child)) {
            continue;
        }
        if (const MujocoBody *body = Object::cast_to<MujocoBody>(child)) {
            const int body_id = mj_name2id(model.get(), mjOBJ_BODY, body->get_mujoco_name().utf8().get_data());
            if (body_id >= 0) {
                body_links.push_back({ body->get_instance_id(), body_id });
            }
        }
        find_bodies(child);
    }
}

// ---- Simulating ----

// The longest stretch of time simulated in one physics frame. If the game falls
// behind by more than this (a long hitch), the rest is dropped instead of being
// caught up, which would make the next frame slower still.
static const double MAX_FRAME_TIME = 0.1;

// Called by Godot every physics frame. Advances the simulation by the frame's
// duration. MuJoCo's timestep is usually much shorter than a physics frame
// (2 ms against 16.7 ms), so this takes several steps; whatever doesn't fit a
// whole number of steps is carried over to the next frame.
void MujocoWorld::_physics_process(double p_delta)
{
    if (Engine::get_singleton()->is_editor_hint() || paused || !model || simulation_error) {
        return;
    }
    const double timestep = model->opt.timestep;
    if (timestep <= 0) {
        return;
    }

    time_accumulator = MIN(time_accumulator + p_delta, MAX_FRAME_TIME);
    // The small allowance keeps rounding from costing a step: 8 steps of 2 ms
    // should fit in a frame that is 16 ms give or take a few bits.
    const int steps = int((time_accumulator + timestep * 1e-6) / timestep);
    if (steps > 0) {
        time_accumulator -= steps * timestep;
        advance(steps);
        sync_poses();
    }
}

// Advances the simulation by p_steps timesteps, whether or not it is paused.
void MujocoWorld::step(int p_steps)
{
    ERR_FAIL_COND_MSG(!model, "Can't step: the model hasn't compiled.");
    ERR_FAIL_COND_MSG(simulation_error, "Can't step: the simulation stopped after a MuJoCo error. Call reset().");
    advance(p_steps);
    sync_poses();
}

// Takes p_steps steps. Returns false if MuJoCo reported an error, after which
// the simulation is stopped.
bool MujocoWorld::advance(int p_steps)
{
    mujoco_take_engine_error(); // forget anything reported before now
    for (int i = 0; i < p_steps; i++) {
        mj_step(model.get(), data.get());
        if (mujoco_take_engine_error()) {
            simulation_error = true;
            ERR_PRINT(String("MuJoCo model '") + String(get_name()) + String("' stopped simulating after an error. Call reset() to start again."));
            return false;
        }
    }
    return true;
}

// Moves every body node to where its body is in the simulation.
void MujocoWorld::sync_poses()
{
    // In the editor the nodes are the authored model; never move them.
    if (Engine::get_singleton()->is_editor_hint()) {
        return;
    }
    // MuJoCo's poses are in the frame of its world body, which is this node.
    const Transform3D world_transform = get_global_transform();
    for (const BodyLink &link : body_links) {
        MujocoBody *body = Object::cast_to<MujocoBody>(ObjectDB::get_instance(link.node_id));
        if (!body || !body->is_inside_tree()) {
            continue; // freed or removed; a recompile is already on its way
        }
        // xpos and xquat hold every body's pose in the world frame.
        const Transform3D pose = mujoco_convert::to_godot(&data->xpos[3 * link.body_id], &data->xquat[4 * link.body_id]);
        body->set_simulated_pose(world_transform * pose);
    }
}

// Puts the simulation back to its initial state, at time zero.
void MujocoWorld::reset()
{
    ERR_FAIL_COND_MSG(!model, "Can't reset: the model hasn't compiled.");
    mj_resetData(model.get(), data.get());
    mj_forward(model.get(), data.get());
    time_accumulator = 0.0;
    simulation_error = false;
    mujoco_take_engine_error();
    sync_poses();
}

// Simulated time in seconds since the last compile or reset.
double MujocoWorld::get_time() const
{
    return data ? data->time : 0.0;
}

bool MujocoWorld::has_simulation_error() const
{
    return simulation_error;
}

void MujocoWorld::set_paused(bool p_paused)
{
    paused = p_paused;
}

bool MujocoWorld::is_paused() const
{
    return paused;
}

// Asks for a recompile at the end of the current frame. Any number of requests
// in one frame result in one compile. Ignored until _ready has made the first
// compile, so loading a scene compiles once, not once per node.
void MujocoWorld::request_recompile()
{
    if (!initialized || recompile_pending) {
        return;
    }
    recompile_pending = true;
    callable_mp(this, &MujocoWorld::compile_if_pending).call_deferred();
}

// The deferred half of request_recompile. If compile() was called directly in
// the meantime, the request has already been met and there is nothing to do.
void MujocoWorld::compile_if_pending()
{
    if (recompile_pending) {
        compile();
    }
}

// The compiled model written out as MJCF (MuJoCo's XML format). Empty if
// nothing has compiled yet.
String MujocoWorld::to_mjcf() const
{
    if (!spec) {
        return String();
    }
    char error[1024] = "";
    // With a buffer that is too small, MuJoCo returns the size it needs.
    char probe[1];
    const int size = mj_saveXMLString(spec.get(), probe, sizeof(probe), error, sizeof(error));
    ERR_FAIL_COND_V_MSG(size < 0, String(), String("Could not write MJCF: ") + String::utf8(error));

    std::string buffer(size + 1, '\0');
    const int result = mj_saveXMLString(spec.get(), buffer.data(), size + 1, error, sizeof(error));
    ERR_FAIL_COND_V_MSG(result != 0, String(), String("Could not write MJCF: ") + String::utf8(error));
    return String::utf8(buffer.c_str());
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
    request_recompile();
    UtilityFunctions::print_verbose("RENAMED: " + get_name());
}

void MujocoWorld::on_child_order_changed() 
{
    request_recompile();
    UtilityFunctions::print_verbose("CHILD ORDER CHANGED: " + get_name());
}

void MujocoWorld::on_disabled() 
{
    UtilityFunctions::print_verbose("DISABLED: " + get_name());
}

void MujocoWorld::on_enabled() 
{
    request_recompile();
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
