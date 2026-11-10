#include "mujoco_world.h"
#include "mujoco_element.h"
#include "mujoco_options.h"

#include <vector>

#include "mujoco_body.h"
#include "mujoco_convert.h"

#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/variant/callable_method_pointer.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

MujocoWorld::MujocoWorld()
{
    mj_defaultOption(&option);

    // Non-persistent: not written into saved scenes.
    add_to_group("mujoco");
}

// Record the error and surface it in the editor. The node stays in the tree so
// the user can fix the offending property; the next successful compile clears it.
void MujocoWorld::report_compile_error(const String &p_error)
{
    compile_error = p_error;
    ERR_PRINT(vformat("MuJoCo model '%s' failed to compile: %s", get_name(), p_error));
    update_configuration_warnings();
}

void MujocoWorld::clear_compile_error()
{
    if (compile_error.is_empty()) {
        return;
    }
    compile_error = String();
    update_configuration_warnings();
}

PackedStringArray MujocoWorld::_get_configuration_warnings() const
{
    PackedStringArray warnings = Node3D::_get_configuration_warnings();
    if (!compile_error.is_empty()) {
        warnings.push_back("MuJoCo model failed to compile:\n" + compile_error);
    }
    return warnings;
}

void MujocoWorld::_bind_methods()
{
    // Options are dynamic properties (see _get_property_list); their constants are bound here.
    mujoco_options::bind_constants(get_class_static());

    ClassDB::bind_method(D_METHOD("get_mjcf"), &MujocoWorld::get_mjcf);
    ClassDB::bind_method(D_METHOD("set_running", "running"), &MujocoWorld::set_running);
    ClassDB::bind_method(D_METHOD("is_running"), &MujocoWorld::is_running);
    ClassDB::bind_method(D_METHOD("set_max_steps_per_frame", "steps"), &MujocoWorld::set_max_steps_per_frame);
    ClassDB::bind_method(D_METHOD("get_max_steps_per_frame"), &MujocoWorld::get_max_steps_per_frame);
    ClassDB::bind_method(D_METHOD("get_simulation_time"), &MujocoWorld::get_simulation_time);
    ClassDB::bind_method(D_METHOD("reset"), &MujocoWorld::reset);
    ClassDB::bind_method(D_METHOD("step", "count"), &MujocoWorld::step, DEFVAL(1));

    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "running"), "set_running", "is_running");
    ADD_PROPERTY(PropertyInfo(Variant::INT, "max_steps_per_frame", PROPERTY_HINT_RANGE, "1,1000,1,or_greater"), "set_max_steps_per_frame", "get_max_steps_per_frame");

    // Emitted after every successful compile. At runtime this means the simulation was reset.
    ADD_SIGNAL(MethodInfo("model_compiled"));
    // Emitted before every MuJoCo step (option_timestep apart), for controllers that
    // should run at the simulation's rate rather than the frame rate.
    ADD_SIGNAL(MethodInfo("pre_step"));
}

bool MujocoWorld::_set(const StringName &p_name, const Variant &p_value)
{
    // Returning true means "this property is ours", not "the value was stored":
    // returning false would make Godot report the property as nonexistent.
    bool accepted = false;
    if (!mujoco_options::set(option, p_name, p_value, accepted)) {
        return false;
    }
    if (!accepted) {
        // Invalid value: already reported by mujoco_options::set, option left unchanged.
        return true;
    }
    if (Engine::get_singleton()->is_editor_hint()) {
        // Recompile so the editor shows any error the new value causes.
        request_compile();
    } else if (model) {
        // MuJoCo options can change on a running model; no reset needed.
        model->opt = option;
        spec->option = option;
    }
    return true;
}

bool MujocoWorld::_get(const StringName &p_name, Variant &r_ret) const
{
    return mujoco_options::get(option, p_name, r_ret);
}

void MujocoWorld::_get_property_list(List<PropertyInfo> *p_list) const
{
    mujoco_options::get_property_list(p_list);
}

bool MujocoWorld::_property_can_revert(const StringName &p_name) const
{
    Variant unused;
    return mujoco_options::get_default(p_name, unused);
}

bool MujocoWorld::_property_get_revert(const StringName &p_name, Variant &r_property) const
{
    return mujoco_options::get_default(p_name, r_property);
}

void MujocoWorld::_notification(int p_what)
{
    switch (p_what) {
        case NOTIFICATION_READY:
            // Called after all children are ready, so the whole tree is in place.
            initialized = true;
            compile_queued = false;
            compile();
            set_physics_process(!Engine::get_singleton()->is_editor_hint());
            break;
        case NOTIFICATION_PHYSICS_PROCESS:
            step_simulation(get_physics_process_delta_time());
            break;
        case NOTIFICATION_PATH_RENAMED:
            // The node name is the model name.
            request_compile();
            break;
        case NOTIFICATION_CHILD_ORDER_CHANGED:
            request_compile();
            break;
    };
}

void MujocoWorld::compile()
{
    std::unique_ptr<mjSpec, MjSpecDeleter> new_spec(mj_makeSpec());
    if (!new_spec) {
        report_compile_error("mj_makeSpec failed to allocate a spec.");
        return;
    }

    const CharString model_name = String(get_name()).utf8();
    mjs_setString(new_spec->modelname, model_name.get_data());
    new_spec->option = option;
    // Angles (joint ranges) are given in radians, like everywhere in Godot's API.
    new_spec->compiler.degree = 0;
    building_bodies.clear();
    add_subtree(this, new_spec.get(), mjs_findBody(new_spec.get(), "world"), Transform3D());

    std::unique_ptr<mjModel, MjModelDeleter> new_model(mj_compile(new_spec.get(), nullptr));
    if (!new_model) {
        report_compile_error(mjs_getError(new_spec.get()));
        return;
    }
    std::unique_ptr<mjData, MjDataDeleter> new_data(mj_makeData(new_model.get()));
    if (!new_data) {
        report_compile_error("mj_makeData failed to allocate simulation data.");
        return;
    }

    // Map body ids to nodes; ids are only assigned by compiling.
    std::vector<uint64_t> new_body_nodes(static_cast<size_t>(new_model->nbody), 0);
    for (const std::pair<mjsBody *, uint64_t> &entry : building_bodies) {
        const int id = mjs_getId(entry.first->element);
        if (id >= 0 && id < new_model->nbody) {
            new_body_nodes[static_cast<size_t>(id)] = entry.second;
        }
    }
    building_bodies.clear();

    // Compute body poses for the initial state.
    mj_forward(new_model.get(), new_data.get());

    // Release the old data before the model it was made from.
    data = std::move(new_data);
    model = std::move(new_model);
    spec = std::move(new_spec);
    body_nodes = std::move(new_body_nodes);
    step_accumulator = 0.0;
    model_generation++;

    // At runtime a recompile resets the simulation: put the bodies back.
    if (!Engine::get_singleton()->is_editor_hint()) {
        sync_bodies();
    }

    clear_compile_error();
    UtilityFunctions::print_verbose(vformat("MuJoCo: compiled model '%s'.", get_name()));
    emit_signal("model_compiled");
}

void MujocoWorld::add_subtree(Node *p_node, mjSpec *p_spec, mjsBody *p_body, const Transform3D &p_to_body)
{
    const int count = p_node->get_child_count();
    for (int i = 0; i < count; i++) {
        // Only Node3D children carry a transform; nested worlds simulate separately.
        Node3D *child = Object::cast_to<Node3D>(p_node->get_child(i));
        if (!child || Object::cast_to<MujocoWorld>(child)) {
            continue;
        }

        MujocoElement *element = Object::cast_to<MujocoElement>(child);
        if (!element) {
            // A plain Node3D: pass its transform on to its descendants.
            add_subtree(child, p_spec, p_body, p_to_body * child->get_transform());
            continue;
        }

        const Transform3D to_body = p_to_body * element->get_model_transform();
        const String name = String(get_path_to(child));
        mjsBody *child_body = element->add_to_spec(p_spec, p_body, to_body, name);
        if (child_body != p_body) {
            building_bodies.emplace_back(child_body, child->get_instance_id());
        }
        // A new body starts a new frame for its descendants.
        add_subtree(child, p_spec, child_body, child_body == p_body ? to_body : Transform3D());
    }
}

void MujocoWorld::step_simulation(double p_delta)
{
    flush_compile();
    if (!running || !model || !data) {
        return;
    }
    const mjtNum timestep = model->opt.timestep;
    if (timestep <= 0.0) {
        return;
    }

    step_accumulator += p_delta;
    // The tolerance stops floating-point drift in the accumulated frame times
    // from occasionally losing a step.
    const double threshold = timestep * (1.0 - 1e-6);
    int steps = 0;
    while (step_accumulator >= threshold && steps < max_steps_per_frame) {
        step_once();
        step_accumulator -= timestep;
        steps++;
    }
    if (step_accumulator >= threshold) {
        UtilityFunctions::print_verbose(vformat("MuJoCo: '%s' hit max_steps_per_frame (%d); dropping %.3f s of backlog.", get_name(), max_steps_per_frame, step_accumulator));
        step_accumulator = 0.0;
    }

    if (steps > 0) {
        // mj_step leaves body poses one step behind the state; bring them up to date.
        mj_kinematics(model.get(), data.get());
        sync_bodies();
    }
}

void MujocoWorld::step_once()
{
    emit_signal("pre_step");
    // A pre_step handler may have caused a compile (by reading state while one was
    // queued), so read the pointers only now.
    if (!model || !data) {
        return;
    }
    stepping = true;
    mj_step(model.get(), data.get());
    stepping = false;
}

void MujocoWorld::reset()
{
    flush_compile();
    ERR_FAIL_COND_MSG(!model || !data, "Nothing to reset: the model has not compiled.");
    mj_resetData(model.get(), data.get());
    step_accumulator = 0.0;
    state_changed();
}

void MujocoWorld::step(int p_count)
{
    ERR_FAIL_COND_MSG(Engine::get_singleton()->is_editor_hint(), "step() only works while the game runs.");
    ERR_FAIL_COND_MSG(p_count < 1, "step() needs a count of at least 1.");
    flush_compile();
    ERR_FAIL_COND_MSG(!model || !data, "Nothing to step: the model has not compiled.");
    for (int i = 0; i < p_count; i++) {
        step_once();
    }
    if (model && data) {
        mj_kinematics(model.get(), data.get());
        sync_bodies();
    }
}

mjModel *MujocoWorld::get_model()
{
    flush_compile();
    return model.get();
}

mjData *MujocoWorld::get_data()
{
    flush_compile();
    return data.get();
}

uint64_t MujocoWorld::get_model_generation() const
{
    return model_generation;
}

void MujocoWorld::state_changed()
{
    if (stepping || !model || !data) {
        return;
    }
    mj_forward(model.get(), data.get());
    if (!Engine::get_singleton()->is_editor_hint()) {
        sync_bodies();
    }
}

void MujocoWorld::sync_bodies()
{
    if (!model || !data) {
        return;
    }
    const Transform3D world_transform = get_global_transform();
    // Body ids are in tree order, so each parent is placed before its children.
    for (int id = 1; id < model->nbody && id < static_cast<int>(body_nodes.size()); id++) {
        Node3D *node = Object::cast_to<Node3D>(ObjectDB::get_instance(body_nodes[static_cast<size_t>(id)]));
        if (!node) {
            continue;
        }
        node->set_global_transform(world_transform * mujoco_convert::to_godot(data->xpos + 3 * id, data->xquat + 4 * id));
    }
}

void MujocoWorld::set_running(bool p_running)
{
    running = p_running;
}

bool MujocoWorld::is_running() const
{
    return running;
}

void MujocoWorld::set_max_steps_per_frame(int p_steps)
{
    ERR_FAIL_COND_MSG(p_steps < 1, "max_steps_per_frame must be at least 1.");
    max_steps_per_frame = p_steps;
}

int MujocoWorld::get_max_steps_per_frame() const
{
    return max_steps_per_frame;
}

double MujocoWorld::get_simulation_time() const
{
    return data ? data->time : 0.0;
}

String MujocoWorld::get_mjcf()
{
    flush_compile();
    if (!spec) {
        return String();
    }

    char error[1024] = "";
    std::vector<char> buffer(16384);
    int result = mj_saveXMLString(spec.get(), buffer.data(), static_cast<int>(buffer.size()), error, sizeof(error));
    if (result > 0) {
        // Buffer too small; result is the required size.
        buffer.resize(static_cast<size_t>(result) + 1);
        result = mj_saveXMLString(spec.get(), buffer.data(), static_cast<int>(buffer.size()), error, sizeof(error));
    }
    ERR_FAIL_COND_V_MSG(result != 0, String(), vformat("Could not save MJCF: %s", error));
    return String::utf8(buffer.data());
}

void MujocoWorld::request_compile()
{
    // Before ready, edits only change node properties; the first compile happens on ready.
    if (!initialized || compile_queued) {
        return;
    }
    compile_queued = true;
    callable_mp(this, &MujocoWorld::flush_compile).call_deferred();
}

void MujocoWorld::flush_compile()
{
    if (!compile_queued) {
        return;
    }
    compile_queued = false;
    compile();
}
