#include "mujoco_body.h"

#include <godot_cpp/classes/engine.hpp>

#include "mujoco_convert.h"
#include "mujoco_world.h"

using namespace godot;

namespace {

// Indices into ATTRIBUTES below. The two must stay in the same order.
enum Attribute {
    MOCAP,
    ATTRIBUTE_COUNT,
};

const MujocoAttribute ATTRIBUTES[] = {
    { "mocap", Variant::BOOL, PROPERTY_HINT_NONE, "" },
};

static_assert(sizeof(ATTRIBUTES) / sizeof(ATTRIBUTES[0]) == ATTRIBUTE_COUNT, "ATTRIBUTES and enum Attribute must match");

} // namespace

void MujocoBody::_bind_methods()
{
    ClassDB::bind_method(D_METHOD("set_applied_force", "force"), &MujocoBody::set_applied_force);
    ClassDB::bind_method(D_METHOD("get_applied_force"), &MujocoBody::get_applied_force);
    ClassDB::bind_method(D_METHOD("set_applied_torque", "torque"), &MujocoBody::set_applied_torque);
    ClassDB::bind_method(D_METHOD("get_applied_torque"), &MujocoBody::get_applied_torque);
    ClassDB::bind_method(D_METHOD("get_linear_velocity"), &MujocoBody::get_linear_velocity);
    ClassDB::bind_method(D_METHOD("get_angular_velocity"), &MujocoBody::get_angular_velocity);

    // State, not settings: shown in the inspector but never saved.
    ADD_GROUP("Simulation State", "");
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "applied_force", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR), "set_applied_force", "get_applied_force");
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "applied_torque", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR), "set_applied_torque", "get_applied_torque");
}

// Lists the attributes here, so the inspector shows them under "MujocoBody".
void MujocoBody::_get_property_list(List<PropertyInfo> *r_list) const
{
    get_attribute_property_list(r_list);
}

const MujocoAttribute *MujocoBody::get_attribute_table(int &r_count) const
{
    r_count = ATTRIBUTE_COUNT;
    return ATTRIBUTES;
}

Variant MujocoBody::read(const mjsBody &p_body, int p_index)
{
    switch (p_index) {
        case MOCAP:
            return bool(p_body.mocap);
    }
    return Variant();
}

void MujocoBody::write(mjsBody &r_body, int p_index, const Variant &p_value)
{
    switch (p_index) {
        case MOCAP:
            r_body.mocap = bool(p_value);
            break;
    }
}

// MuJoCo's built-in defaults.
Variant MujocoBody::get_inherited_value(int p_index) const
{
    mjsBody defaults;
    mjs_defaultBody(&defaults);
    return read(defaults, p_index);
}

mjsBody *MujocoBody::add_to_spec(mjSpec *p_spec, mjsBody *p_body, const Transform3D &p_transform) const
{
    // nullptr: no default class yet, so the body starts from MuJoCo's defaults.
    mjsBody *body = mjs_addBody(p_body, nullptr);
    mjs_setName(body->element, get_mujoco_name().utf8().get_data());
    mujoco_convert::to_mujoco(p_transform, body->pos, body->quat);

    for (int i = 0; i < ATTRIBUTE_COUNT; i++) {
        Variant value;
        if (get_override(i, value)) {
            write(*body, i, value);
        }
    }

    // This node's children are added to the new body.
    return body;
}

// ---- Rest pose, and being moved ----

Transform3D MujocoBody::get_spec_transform() const
{
    return simulated ? rest_transform : get_transform();
}

void MujocoBody::set_simulated_pose(const Transform3D &p_global_transform)
{
    if (!simulated) {
        rest_transform = get_transform();
        simulated = true;
    }
    applying_simulation_pose = true;
    set_global_transform(p_global_transform);
    applying_simulation_pose = false;
}

MujocoWorld *MujocoBody::get_simulating_world() const
{
    MujocoWorld *world = get_world();
    if (!world || !world->get_model() || get_mujoco_id() < 0) {
        return nullptr;
    }
    return world;
}

int MujocoBody::get_mocap_id() const
{
    const MujocoWorld *world = get_simulating_world();
    // The compiled model, not the attribute, says whether the body in the
    // running simulation is a mocap body.
    return world ? world->get_model()->body_mocapid[get_mujoco_id()] : -1;
}

// The node was moved by something other than the simulation.
void MujocoBody::transform_changed_by_user()
{
    const int mocap_id = get_mocap_id();
    if (mocap_id >= 0 && !Engine::get_singleton()->is_editor_hint()) {
        // A mocap body in a running game: move it in the simulation. MuJoCo
        // keeps mocap poses in its own arrays, in the world body's frame,
        // which is the frame of the MujocoWorld node.
        MujocoWorld *world = get_simulating_world();
        const Transform3D pose = world->get_global_transform().affine_inverse() * get_global_transform();
        mujoco_convert::to_mujoco(pose, &world->get_data()->mocap_pos[3 * mocap_id], &world->get_data()->mocap_quat[4 * mocap_id]);
        world->state_changed();
        return;
    }

    // Anything else places the body: where it is now is its new rest pose, and
    // the model has to be rebuilt around it.
    simulated = false;
    request_recompile();
}

// ---- Simulation state ----

// Forces and velocities cross between Godot's global frame and the simulation's
// frame, which is the MujocoWorld node's. Only its rotation matters for these.

void MujocoBody::set_applied_force(const Vector3 &p_force)
{
    MujocoWorld *world = get_simulating_world();
    ERR_FAIL_COND_MSG(!world, "Can't set applied_force: the body is not in a compiled model.");
    // MuJoCo keeps one external force and torque per body, six numbers: force
    // first, then torque.
    const Basis to_simulation = world->get_global_transform().basis.orthonormalized().inverse();
    mujoco_convert::to_mujoco(to_simulation.xform(p_force), &world->get_data()->xfrc_applied[6 * get_mujoco_id()]);
}

Vector3 MujocoBody::get_applied_force() const
{
    const MujocoWorld *world = get_simulating_world();
    if (!world) {
        return Vector3();
    }
    const Basis to_global = world->get_global_transform().basis.orthonormalized();
    return to_global.xform(mujoco_convert::to_godot(&world->get_data()->xfrc_applied[6 * get_mujoco_id()]));
}

void MujocoBody::set_applied_torque(const Vector3 &p_torque)
{
    MujocoWorld *world = get_simulating_world();
    ERR_FAIL_COND_MSG(!world, "Can't set applied_torque: the body is not in a compiled model.");
    // A torque is an axis and a strength, and its axis converts like any vector.
    const Basis to_simulation = world->get_global_transform().basis.orthonormalized().inverse();
    mujoco_convert::to_mujoco(to_simulation.xform(p_torque), &world->get_data()->xfrc_applied[6 * get_mujoco_id() + 3]);
}

Vector3 MujocoBody::get_applied_torque() const
{
    const MujocoWorld *world = get_simulating_world();
    if (!world) {
        return Vector3();
    }
    const Basis to_global = world->get_global_transform().basis.orthonormalized();
    return to_global.xform(mujoco_convert::to_godot(&world->get_data()->xfrc_applied[6 * get_mujoco_id() + 3]));
}

Vector3 MujocoBody::get_linear_velocity() const
{
    const MujocoWorld *world = get_simulating_world();
    if (!world) {
        return Vector3();
    }
    // Six numbers: rotational velocity first, then linear. The 0 asks for them
    // in the world's frame rather than the body's own.
    mjtNum velocity[6];
    mj_objectVelocity(world->get_model(), world->get_data(), mjOBJ_BODY, get_mujoco_id(), velocity, 0);
    const Basis to_global = world->get_global_transform().basis.orthonormalized();
    return to_global.xform(mujoco_convert::to_godot(&velocity[3]));
}

Vector3 MujocoBody::get_angular_velocity() const
{
    const MujocoWorld *world = get_simulating_world();
    if (!world) {
        return Vector3();
    }
    mjtNum velocity[6];
    mj_objectVelocity(world->get_model(), world->get_data(), mjOBJ_BODY, get_mujoco_id(), velocity, 0);
    const Basis to_global = world->get_global_transform().basis.orthonormalized();
    return to_global.xform(mujoco_convert::to_godot(&velocity[0]));
}

// The state properties are greyed out in the editor, where nothing is simulating.
void MujocoBody::_validate_property(PropertyInfo &p_property) const
{
    if (p_property.name != StringName("applied_force") && p_property.name != StringName("applied_torque")) {
        return;
    }
    if (Engine::get_singleton()->is_editor_hint()) {
        p_property.usage |= PROPERTY_USAGE_READ_ONLY;
    }
}
