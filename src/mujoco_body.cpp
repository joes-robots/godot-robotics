#include "mujoco_body.h"
#include "mujoco_convert.h"
#include "mujoco_joint.h"
#include "mujoco_world.h"

#include <godot_cpp/classes/engine.hpp>

using namespace godot;

mjsBody *MujocoBody::add_to_spec(mjSpec * /*p_spec*/, mjsBody *p_body, const Transform3D &p_transform, const String &p_name)
{
    mjsBody *body = mjs_addBody(p_body, nullptr);
    mjs_setName(body->element, p_name.utf8().get_data());
    mujoco_convert::to_mujoco(p_transform, body->pos, body->quat);
    body->mocap = mocap ? 1 : 0;
    if (explicit_inertia) {
        body->explicitinertial = 1;
        body->mass = mass;
        // The principal axes as a frame, in MuJoCo's convention (like a geom's frame).
        mujoco_convert::to_mujoco(Transform3D(Basis(inertia_orientation), center_of_mass), body->ipos, body->iquat);
        // Moments about Godot's x, y, z axes of that frame are MuJoCo's x, z, y.
        body->inertia[0] = inertia.x;
        body->inertia[1] = inertia.z;
        body->inertia[2] = inertia.y;
    }
    // Only for the MJCF: elements already name their resolved classes.
    if (!child_class.is_empty()) {
        mjs_setString(body->childclass, String(child_class).utf8().get_data());
    }
    return body;
}

void MujocoBody::set_child_class(const StringName &p_class)
{
    child_class = p_class;
    update_configuration_warnings();
    if (MujocoWorld *world = get_world()) {
        world->default_classes_changed();
    }
}

StringName MujocoBody::get_child_class() const
{
    return child_class;
}

void MujocoBody::_validate_property(PropertyInfo &p_property) const
{
    const StringName name = p_property.name;
    if (!explicit_inertia && (name == StringName("mass") || name == StringName("center_of_mass") ||
                                     name == StringName("inertia") || name == StringName("inertia_orientation"))) {
        p_property.usage = PROPERTY_USAGE_NO_EDITOR;
    }
    if (p_property.name == StringName("child_class")) {
        if (const MujocoWorld *world = get_world()) {
            p_property.hint = PROPERTY_HINT_ENUM_SUGGESTION;
            p_property.hint_string = String(",").join(world->get_default_class_names());
        }
    }
}

Transform3D MujocoBody::get_model_transform()
{
    if (Engine::get_singleton()->is_editor_hint()) {
        return get_transform();
    }
    // Captured lazily, on the first compile that includes this body, so a script
    // can still position the body in the frame it adds it.
    if (!has_rest_transform) {
        rest_transform = get_transform();
        has_rest_transform = true;
    }
    return rest_transform;
}

PackedStringArray MujocoBody::_get_configuration_warnings() const
{
    PackedStringArray warnings = MujocoElement::_get_configuration_warnings();
    const MujocoWorld *world = get_world();
    if (world && !child_class.is_empty() && !world->find_default_class(child_class)) {
        warnings.push_back(vformat("There is no default class \"%s\" in this world.", child_class));
    }
    if (!mocap) {
        return warnings;
    }
    for (const Node *node = get_parent(); node && node != world; node = node->get_parent()) {
        if (Object::cast_to<MujocoBody>(node)) {
            warnings.push_back("A mocap body must not be inside another MujocoBody.");
            break;
        }
    }
    for (int i = 0; i < get_child_count(); i++) {
        if (Object::cast_to<MujocoJoint>(get_child(i))) {
            warnings.push_back("A mocap body cannot have joints; scripts move it directly.");
            break;
        }
    }
    return warnings;
}

void MujocoBody::_notification(int p_what)
{
    if (p_what == NOTIFICATION_EXIT_TREE) {
        // A new parent means a new reference frame; capture again next time.
        has_rest_transform = false;
    }
}

void MujocoBody::set_mocap(bool p_mocap)
{
    mocap = p_mocap;
    update_configuration_warnings();
    request_compile();
}

bool MujocoBody::is_mocap() const
{
    return mocap;
}

int MujocoBody::get_body_id(const char *p_action)
{
    const int id = get_model_id(mjOBJ_BODY);
    ERR_FAIL_COND_V_MSG(id < 0, -1, vformat("Cannot %s: this body is not part of a compiled MuJoCo model.", p_action));
    return id;
}

Vector3 MujocoBody::get_linear_velocity()
{
    const int id = get_body_id("read the velocity");
    if (id < 0) {
        return Vector3();
    }
    MujocoWorld *world = get_world();
    mjtNum velocity[6]; // angular, then linear
    // XBODY: at the body's origin (BODY would be at its center of mass).
    mj_objectVelocity(world->get_model(), world->get_data(), mjOBJ_XBODY, id, velocity, 0);
    return world->mujoco_to_global_vector(velocity + 3);
}

Vector3 MujocoBody::get_angular_velocity()
{
    const int id = get_body_id("read the velocity");
    if (id < 0) {
        return Vector3();
    }
    MujocoWorld *world = get_world();
    mjtNum velocity[6];
    mj_objectVelocity(world->get_model(), world->get_data(), mjOBJ_XBODY, id, velocity, 0);
    return world->mujoco_to_global_vector(velocity);
}

bool MujocoBody::get_free_joint_index(int &r_qpos, int &r_dof)
{
    ERR_FAIL_COND_V_MSG(Engine::get_singleton()->is_editor_hint(), false, "Body state can only be set while the game runs.");
    const int id = get_body_id("set the state");
    if (id < 0) {
        return false;
    }
    const mjModel *model = get_world()->get_model();
    const int joint = model->body_jntadr[id];
    ERR_FAIL_COND_V_MSG(model->body_jntnum[id] != 1 || model->jnt_type[joint] != mjJNT_FREE, false,
            "Setting a body's pose or velocity needs a free joint as its only joint. Use the joint's own methods otherwise.");
    r_qpos = model->jnt_qposadr[joint];
    r_dof = model->jnt_dofadr[joint];
    return true;
}

void MujocoBody::set_linear_velocity(const Vector3 &p_velocity)
{
    int qpos = 0, dof = 0;
    if (!get_free_joint_index(qpos, dof)) {
        return;
    }
    MujocoWorld *world = get_world();
    // A free joint's linear velocity is that of the body's origin, in the world frame.
    world->global_to_mujoco_vector(p_velocity, world->get_data()->qvel + dof);
    world->state_changed();
}

void MujocoBody::set_angular_velocity(const Vector3 &p_velocity)
{
    int qpos = 0, dof = 0;
    if (!get_free_joint_index(qpos, dof)) {
        return;
    }
    MujocoWorld *world = get_world();
    mjData *data = world->get_data();
    // A free joint's angular velocity is in the body's local frame.
    mjtNum world_velocity[3];
    world->global_to_mujoco_vector(p_velocity, world_velocity);
    mju_mulMatTVec3(data->qvel + dof + 3, data->xmat + 9 * get_model_id(mjOBJ_BODY), world_velocity);
    world->state_changed();
}

void MujocoBody::teleport(const Transform3D &p_transform)
{
    int qpos = 0, dof = 0;
    if (!get_free_joint_index(qpos, dof)) {
        return;
    }
    MujocoWorld *world = get_world();
    mjData *data = world->get_data();
    // A free joint's position is the body's position and orientation in the world frame.
    world->global_to_mujoco_transform(p_transform, data->qpos + qpos, data->qpos + qpos + 3);
    world->state_changed();
}

void MujocoBody::add_wrench(const Vector3 &p_force, const Vector3 *p_position, const Vector3 &p_torque)
{
    ERR_FAIL_COND_MSG(Engine::get_singleton()->is_editor_hint(), "Forces can only be applied while the game runs.");
    const int id = get_body_id("apply a force");
    if (id < 0) {
        return;
    }
    MujocoWorld *world = get_world();
    const mjData *data = world->get_data();

    mjtNum force[3], torque[3];
    world->global_to_mujoco_vector(p_force, force);
    world->global_to_mujoco_vector(p_torque, torque);
    if (p_position) {
        // MuJoCo applies the force at the center of mass; a force elsewhere adds torque r x F.
        mjtNum offset[3], arm[3], extra[3];
        world->global_to_mujoco_vector(*p_position, offset);
        for (int i = 0; i < 3; i++) {
            arm[i] = data->xpos[3 * id + i] + offset[i] - data->xipos[3 * id + i];
        }
        mju_cross(extra, arm, force);
        for (int i = 0; i < 3; i++) {
            torque[i] += extra[i];
        }
    }
    world->add_body_wrench(id, force, torque);
}

void MujocoBody::apply_force(const Vector3 &p_force, const Vector3 &p_position)
{
    add_wrench(p_force, &p_position, Vector3());
}

void MujocoBody::apply_central_force(const Vector3 &p_force)
{
    add_wrench(p_force, nullptr, Vector3());
}

void MujocoBody::apply_torque(const Vector3 &p_torque)
{
    add_wrench(Vector3(), nullptr, p_torque);
}

void MujocoBody::_bind_methods()
{
    ClassDB::bind_method(D_METHOD("set_mocap", "mocap"), &MujocoBody::set_mocap);
    ClassDB::bind_method(D_METHOD("is_mocap"), &MujocoBody::is_mocap);
    ClassDB::bind_method(D_METHOD("get_linear_velocity"), &MujocoBody::get_linear_velocity);
    ClassDB::bind_method(D_METHOD("get_angular_velocity"), &MujocoBody::get_angular_velocity);
    ClassDB::bind_method(D_METHOD("set_linear_velocity", "velocity"), &MujocoBody::set_linear_velocity);
    ClassDB::bind_method(D_METHOD("set_angular_velocity", "velocity"), &MujocoBody::set_angular_velocity);
    ClassDB::bind_method(D_METHOD("teleport", "transform"), &MujocoBody::teleport);
    ClassDB::bind_method(D_METHOD("apply_force", "force", "position"), &MujocoBody::apply_force, DEFVAL(Vector3()));
    ClassDB::bind_method(D_METHOD("apply_central_force", "force"), &MujocoBody::apply_central_force);
    ClassDB::bind_method(D_METHOD("apply_torque", "torque"), &MujocoBody::apply_torque);

    ClassDB::bind_method(D_METHOD("set_child_class", "class"), &MujocoBody::set_child_class);
    ClassDB::bind_method(D_METHOD("get_child_class"), &MujocoBody::get_child_class);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "mocap"), "set_mocap", "is_mocap");
    ADD_PROPERTY(PropertyInfo(Variant::STRING_NAME, "child_class"), "set_child_class", "get_child_class");

    ClassDB::bind_method(D_METHOD("set_explicit_inertia", "explicit"), &MujocoBody::set_explicit_inertia);
    ClassDB::bind_method(D_METHOD("has_explicit_inertia"), &MujocoBody::has_explicit_inertia);
    ClassDB::bind_method(D_METHOD("set_mass", "mass"), &MujocoBody::set_mass);
    ClassDB::bind_method(D_METHOD("get_mass"), &MujocoBody::get_mass);
    ClassDB::bind_method(D_METHOD("set_center_of_mass", "center"), &MujocoBody::set_center_of_mass);
    ClassDB::bind_method(D_METHOD("get_center_of_mass"), &MujocoBody::get_center_of_mass);
    ClassDB::bind_method(D_METHOD("set_inertia", "inertia"), &MujocoBody::set_inertia);
    ClassDB::bind_method(D_METHOD("get_inertia"), &MujocoBody::get_inertia);
    ClassDB::bind_method(D_METHOD("set_inertia_orientation", "orientation"), &MujocoBody::set_inertia_orientation);
    ClassDB::bind_method(D_METHOD("get_inertia_orientation"), &MujocoBody::get_inertia_orientation);
    ADD_GROUP("Inertia", "");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "explicit_inertia"), "set_explicit_inertia", "has_explicit_inertia");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "mass", PROPERTY_HINT_RANGE, "0.001,1000,0.001,or_greater,exp,suffix:kg"), "set_mass", "get_mass");
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "center_of_mass", PROPERTY_HINT_NONE, "suffix:m"), "set_center_of_mass", "get_center_of_mass");
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "inertia", PROPERTY_HINT_NONE, "suffix:kg m^2"), "set_inertia", "get_inertia");
    ADD_PROPERTY(PropertyInfo(Variant::QUATERNION, "inertia_orientation"), "set_inertia_orientation", "get_inertia_orientation");
}

void MujocoBody::set_explicit_inertia(bool p_explicit)
{
    explicit_inertia = p_explicit;
    notify_property_list_changed();
    request_compile();
}

bool MujocoBody::has_explicit_inertia() const { return explicit_inertia; }

void MujocoBody::set_mass(double p_mass)
{
    ERR_FAIL_COND_MSG(p_mass < 0.0, "Mass must not be negative.");
    mass = p_mass;
    request_compile();
}

double MujocoBody::get_mass() const { return mass; }

void MujocoBody::set_center_of_mass(const Vector3 &p_center)
{
    center_of_mass = p_center;
    request_compile();
}

Vector3 MujocoBody::get_center_of_mass() const { return center_of_mass; }

void MujocoBody::set_inertia(const Vector3 &p_inertia)
{
    ERR_FAIL_COND_MSG(p_inertia.x < 0 || p_inertia.y < 0 || p_inertia.z < 0, "Inertia must not be negative.");
    inertia = p_inertia;
    request_compile();
}

Vector3 MujocoBody::get_inertia() const { return inertia; }

void MujocoBody::set_inertia_orientation(const Quaternion &p_orientation)
{
    ERR_FAIL_COND_MSG(!p_orientation.is_normalized(), "The inertia orientation must be a unit quaternion.");
    inertia_orientation = p_orientation;
    request_compile();
}

Quaternion MujocoBody::get_inertia_orientation() const { return inertia_orientation; }
