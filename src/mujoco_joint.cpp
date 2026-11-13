#include "mujoco_joint.h"

#include "mujoco_convert.h"
#include <godot_cpp/classes/engine.hpp>

#include "mujoco_geom.h"
#include "mujoco_world.h"

using namespace godot;

namespace {

// Indices into ATTRIBUTES below. The two must stay in the same order.
enum Attribute {
    TYPE,
    AXIS,
    LIMITED,
    RANGE_LOWER,
    RANGE_UPPER,
    REF,
    SPRINGREF,
    STIFFNESS,
    DAMPING,
    ARMATURE,
    FRICTIONLOSS,
    ATTRIBUTE_COUNT,
};

const MujocoAttribute ATTRIBUTES[] = {
    { "type", Variant::INT, PROPERTY_HINT_ENUM, "Free:0,Ball:1,Slide:2,Hinge:3" },
    { "axis", Variant::VECTOR3, PROPERTY_HINT_NONE, "" },
    { "limited", Variant::BOOL, PROPERTY_HINT_NONE, "" },
    { "range_lower", Variant::FLOAT, PROPERTY_HINT_NONE, "" },
    { "range_upper", Variant::FLOAT, PROPERTY_HINT_NONE, "" },
    { "ref", Variant::FLOAT, PROPERTY_HINT_NONE, "" },
    { "springref", Variant::FLOAT, PROPERTY_HINT_NONE, "" },
    { "stiffness", Variant::FLOAT, PROPERTY_HINT_NONE, "" },
    { "damping", Variant::FLOAT, PROPERTY_HINT_NONE, "" },
    { "armature", Variant::FLOAT, PROPERTY_HINT_NONE, "" },
    { "frictionloss", Variant::FLOAT, PROPERTY_HINT_NONE, "" },
};

static_assert(sizeof(ATTRIBUTES) / sizeof(ATTRIBUTES[0]) == ATTRIBUTE_COUNT, "ATTRIBUTES and enum Attribute must match");
static_assert(mjJNT_FREE == 0 && mjJNT_BALL == 1 && mjJNT_SLIDE == 2 && mjJNT_HINGE == 3);

// Inspector hint that shows a value stored in radians as degrees.
const char *const DEGREES_HINT = "-360,360,0.1,or_less,or_greater,radians_as_degrees";

} // namespace

void MujocoJoint::_bind_methods()
{
    BIND_ENUM_CONSTANT(TYPE_FREE);
    BIND_ENUM_CONSTANT(TYPE_BALL);
    BIND_ENUM_CONSTANT(TYPE_SLIDE);
    BIND_ENUM_CONSTANT(TYPE_HINGE);

    ClassDB::bind_method(D_METHOD("get_joint_position"), &MujocoJoint::get_joint_position);
    ClassDB::bind_method(D_METHOD("set_joint_position", "position"), &MujocoJoint::set_joint_position);
    ClassDB::bind_method(D_METHOD("get_joint_velocity"), &MujocoJoint::get_joint_velocity);
    ClassDB::bind_method(D_METHOD("set_joint_velocity", "velocity"), &MujocoJoint::set_joint_velocity);
    ClassDB::bind_method(D_METHOD("get_applied_force"), &MujocoJoint::get_applied_force);
    ClassDB::bind_method(D_METHOD("set_applied_force", "force"), &MujocoJoint::set_applied_force);
    ClassDB::bind_method(D_METHOD("get_qpos_address"), &MujocoJoint::get_qpos_address);
    ClassDB::bind_method(D_METHOD("get_qvel_address"), &MujocoJoint::get_qvel_address);

    // State, not settings: shown in the inspector (PROPERTY_USAGE_EDITOR) but
    // never saved with the scene.
    ADD_GROUP("Simulation State", "");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "joint_position", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR), "set_joint_position", "get_joint_position");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "joint_velocity", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR), "set_joint_velocity", "get_joint_velocity");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "applied_force", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR), "set_applied_force", "get_applied_force");
}

// Lists the attributes here, so the inspector shows them under "MujocoJoint".
void MujocoJoint::_get_property_list(List<PropertyInfo> *r_list) const
{
    get_attribute_property_list(r_list);
}

const MujocoAttribute *MujocoJoint::get_attribute_table(int &r_count) const
{
    r_count = ATTRIBUTE_COUNT;
    return ATTRIBUTES;
}

// Which attributes mean something for which joint type:
//
//                 free   ball   slide  hinge
//   axis           -      -      yes    yes
//   limited        -      yes    yes    yes
//   range_lower    -      -      (1)    (1)
//   range_upper    -      (1)    (1)    (1)
//   ref            -      -      yes    yes
//   springref      -      -      yes    yes
//   the rest       yes    yes    yes    yes
//
// (1) only while `limited` is on. A ball joint's limit is a single largest
// angle, range_upper; its lower limit is always 0.
bool MujocoJoint::applies(int p_index) const
{
    const int type = get_attribute(TYPE);
    switch (p_index) {
        case AXIS:
        case REF:
        case SPRINGREF:
            return type == TYPE_SLIDE || type == TYPE_HINGE;
        case LIMITED:
            return type != TYPE_FREE;
        case RANGE_LOWER:
            return (type == TYPE_SLIDE || type == TYPE_HINGE) && bool(get_attribute(LIMITED));
        case RANGE_UPPER:
            return type != TYPE_FREE && bool(get_attribute(LIMITED));
    }
    return true;
}

void MujocoJoint::customize_attribute_property(int p_index, PropertyInfo &r_property) const
{
    // Greyed out: shown, but not editable.
    if (!applies(p_index)) {
        r_property.usage |= PROPERTY_USAGE_READ_ONLY;
    }

    // These are angles on a hinge or ball joint and lengths on a slide.
    const int type = get_attribute(TYPE);
    const bool is_angle = type == TYPE_HINGE || type == TYPE_BALL;
    if (is_angle && (p_index == RANGE_LOWER || p_index == RANGE_UPPER || p_index == REF || p_index == SPRINGREF)) {
        r_property.hint = PROPERTY_HINT_RANGE;
        r_property.hint_string = DEGREES_HINT;
    }
}

// Changing the type changes which attributes are greyed out and which are
// angles; `limited` greys the range in and out.
bool MujocoJoint::changes_attribute_presentation(int p_index) const
{
    return p_index == TYPE || p_index == LIMITED;
}

Variant MujocoJoint::read(const mjsJoint &p_joint, int p_index)
{
    switch (p_index) {
        case TYPE:
            return int(p_joint.type);
        case AXIS:
            return mujoco_convert::to_godot(p_joint.axis);
        case LIMITED:
            // MuJoCo's own default is "automatic": limited if a range is given.
            if (p_joint.limited == mjLIMITED_AUTO) {
                return p_joint.range[0] < p_joint.range[1];
            }
            return p_joint.limited == mjLIMITED_TRUE;
        case RANGE_LOWER:
            return p_joint.range[0];
        case RANGE_UPPER:
            return p_joint.range[1];
        case REF:
            return p_joint.ref;
        case SPRINGREF:
            return p_joint.springref;
        case STIFFNESS:
            // MuJoCo allows stiffness and damping that vary with position, as
            // polynomial coefficients. Entry 0 is the ordinary constant one.
            return p_joint.stiffness[0];
        case DAMPING:
            return p_joint.damping[0];
        case ARMATURE:
            return p_joint.armature;
        case FRICTIONLOSS:
            return p_joint.frictionloss;
    }
    return Variant();
}

void MujocoJoint::write(mjsJoint &r_joint, int p_index, const Variant &p_value)
{
    switch (p_index) {
        case TYPE:
            r_joint.type = mjtJoint(int(p_value));
            break;
        case AXIS:
            mujoco_convert::to_mujoco(Vector3(p_value), r_joint.axis);
            break;
        case LIMITED:
            r_joint.limited = bool(p_value) ? mjLIMITED_TRUE : mjLIMITED_FALSE;
            break;
        case RANGE_LOWER:
            r_joint.range[0] = p_value;
            break;
        case RANGE_UPPER:
            r_joint.range[1] = p_value;
            break;
        case REF:
            r_joint.ref = p_value;
            break;
        case SPRINGREF:
            r_joint.springref = p_value;
            break;
        case STIFFNESS:
            r_joint.stiffness[0] = p_value;
            break;
        case DAMPING:
            r_joint.damping[0] = p_value;
            break;
        case ARMATURE:
            r_joint.armature = p_value;
            break;
        case FRICTIONLOSS:
            r_joint.frictionloss = p_value;
            break;
    }
}

// MuJoCo's built-in defaults. Note the default type is a hinge, as in MJCF.
Variant MujocoJoint::get_inherited_value(int p_index) const
{
    mjsJoint defaults;
    mjs_defaultJoint(&defaults);
    return read(defaults, p_index);
}

mjsBody *MujocoJoint::add_to_spec(mjSpec *p_spec, mjsBody *p_body, const Transform3D &p_transform) const
{
    mjsJoint *joint = mjs_addJoint(p_body, nullptr);
    mjs_setName(joint->element, get_mujoco_name().utf8().get_data());
    // The node's position is the joint's anchor point in its body.
    mujoco_convert::to_mujoco(p_transform.origin, joint->pos);

    // Overridden attributes that apply to this joint's type are written.
    for (int i = 0; i < ATTRIBUTE_COUNT; i++) {
        Variant value;
        if (get_override(i, value) && applies(i)) {
            write(*joint, i, value);
        }
    }

    // MuJoCo refuses to compile a limited joint whose range is empty (lower not
    // below upper). That is exactly the state a joint is in just after
    // `limited` is ticked and before a range is typed in, so until the range
    // is usable the joint is left unlimited instead of being an error.
    if (joint->limited == mjLIMITED_TRUE && !(joint->range[0] < joint->range[1])) {
        joint->limited = mjLIMITED_FALSE;
    }

    // The axis attribute is in this node's frame; MuJoCo wants it in the
    // body's. p_transform is this node's pose in the body, so its rotation
    // takes one to the other. Nothing to do for an unrotated node with no
    // axis set: MuJoCo's default already stands.
    const Basis rotation = p_transform.basis.orthonormalized();
    if (applies(AXIS) && (has_override("axis") || !rotation.is_equal_approx(Basis()))) {
        const Vector3 axis = get_attribute(AXIS);
        mujoco_convert::to_mujoco(rotation.xform(axis), joint->axis);
    }

    // A joint can't contain other elements.
    return nullptr;
}

// ---- Editor gizmo ----

// The gizmo itself is drawn by MujocoJointGizmoPlugin; update_gizmos() asks the
// editor to redraw it, and does nothing in a running game.

void MujocoJoint::attributes_changed()
{
    update_gizmos();
}

void MujocoJoint::_notification(int p_what)
{
    if (p_what == NOTIFICATION_ENTER_TREE) {
        update_gizmos();
    }
}

float MujocoJoint::get_gizmo_size() const
{
    // The farthest any geom on the same body reaches from this joint's anchor.
    float reach = 0.0f;
    if (const Node *parent = get_parent()) {
        for (int i = 0; i < parent->get_child_count(); i++) {
            if (const MujocoGeom *geom = Object::cast_to<MujocoGeom>(parent->get_child(i))) {
                reach = MAX(reach, get_position().distance_to(geom->get_position()) + geom->get_bounding_radius());
            }
        }
    }
    // Half as long again, so the gizmo's ends are clear of the geoms; and never
    // so small that it is hard to see or click.
    return MAX(1.5f * reach, 0.25f);
}

// ---- Simulation state ----

bool MujocoJoint::has_scalar_state() const
{
    const int type = get_attribute(TYPE);
    return type == TYPE_HINGE || type == TYPE_SLIDE;
}

MujocoWorld *MujocoJoint::get_simulating_world() const
{
    MujocoWorld *world = get_world();
    if (!world || !world->get_model() || get_mujoco_id() < 0) {
        return nullptr;
    }
    return world;
}

// MuJoCo keeps every joint's position in one array, qpos, and every velocity in
// another, qvel. The model records where each joint's entries start.
int MujocoJoint::get_qpos_address() const
{
    const MujocoWorld *world = get_simulating_world();
    return world ? world->get_model()->jnt_qposadr[get_mujoco_id()] : -1;
}

int MujocoJoint::get_qvel_address() const
{
    const MujocoWorld *world = get_simulating_world();
    return world ? world->get_model()->jnt_dofadr[get_mujoco_id()] : -1;
}

double MujocoJoint::get_joint_position() const
{
    const MujocoWorld *world = get_simulating_world();
    if (!world || !has_scalar_state()) {
        return 0.0;
    }
    return world->get_data()->qpos[get_qpos_address()];
}

void MujocoJoint::set_joint_position(double p_position)
{
    MujocoWorld *world = get_simulating_world();
    ERR_FAIL_COND_MSG(!world, "Can't set joint_position: the joint is not in a compiled model.");
    ERR_FAIL_COND_MSG(!has_scalar_state(), "joint_position is only for hinge and slide joints. Use MujocoWorld.set_qpos for ball and free joints.");
    world->get_data()->qpos[get_qpos_address()] = p_position;
    world->state_changed();
}

double MujocoJoint::get_joint_velocity() const
{
    const MujocoWorld *world = get_simulating_world();
    if (!world || !has_scalar_state()) {
        return 0.0;
    }
    return world->get_data()->qvel[get_qvel_address()];
}

void MujocoJoint::set_joint_velocity(double p_velocity)
{
    MujocoWorld *world = get_simulating_world();
    ERR_FAIL_COND_MSG(!world, "Can't set joint_velocity: the joint is not in a compiled model.");
    ERR_FAIL_COND_MSG(!has_scalar_state(), "joint_velocity is only for hinge and slide joints. Use MujocoWorld.set_qvel for ball and free joints.");
    world->get_data()->qvel[get_qvel_address()] = p_velocity;
    world->state_changed();
}

// MuJoCo keeps one applied force per degree of freedom, in qfrc_applied, laid
// out like qvel.
double MujocoJoint::get_applied_force() const
{
    const MujocoWorld *world = get_simulating_world();
    if (!world || !has_scalar_state()) {
        return 0.0;
    }
    return world->get_data()->qfrc_applied[get_qvel_address()];
}

void MujocoJoint::set_applied_force(double p_force)
{
    MujocoWorld *world = get_simulating_world();
    ERR_FAIL_COND_MSG(!world, "Can't set applied_force: the joint is not in a compiled model.");
    ERR_FAIL_COND_MSG(!has_scalar_state(), "applied_force is only for hinge and slide joints.");
    world->get_data()->qfrc_applied[get_qvel_address()] = p_force;
}

// Adjusts how the state properties appear in the inspector. They are greyed
// out where they can't be used: on ball and free joints, and in the editor,
// where nothing is simulating. On a hinge, position and velocity are angles.
void MujocoJoint::_validate_property(PropertyInfo &p_property) const
{
    const bool is_motion = p_property.name == StringName("joint_position") || p_property.name == StringName("joint_velocity");
    if (!is_motion && p_property.name != StringName("applied_force")) {
        return;
    }
    if (!has_scalar_state() || Engine::get_singleton()->is_editor_hint()) {
        p_property.usage |= PROPERTY_USAGE_READ_ONLY;
    }
    if (is_motion && int(get_attribute(TYPE)) == TYPE_HINGE) {
        p_property.hint = PROPERTY_HINT_RANGE;
        p_property.hint_string = DEGREES_HINT;
    }
}

// ---- Live attributes ----

// Damping, friction loss, armature and stiffness are plain numbers in the
// compiled model, and so are the limits. The first three are stored per degree
// of freedom: one for a hinge or slide, three for a ball, six for a free joint.
bool MujocoJoint::write_live(int p_index, MujocoWorld &p_world) const
{
    mjModel *model = p_world.get_model();
    const int id = get_mujoco_id();
    const int first_dof = model->jnt_dofadr[id];
    const int dof_count = model->jnt_type[id] == mjJNT_FREE ? 6 : (model->jnt_type[id] == mjJNT_BALL ? 3 : 1);

    switch (p_index) {
        case DAMPING:
            for (int i = 0; i < dof_count; i++) {
                model->dof_damping[first_dof + i] = get_attribute(DAMPING);
            }
            return true;
        case FRICTIONLOSS:
            for (int i = 0; i < dof_count; i++) {
                model->dof_frictionloss[first_dof + i] = get_attribute(FRICTIONLOSS);
            }
            return true;
        case ARMATURE:
            for (int i = 0; i < dof_count; i++) {
                model->dof_armature[first_dof + i] = get_attribute(ARMATURE);
            }
            // Armature is inertia, and MuJoCo keeps quantities derived from
            // the model's inertia that have to be worked out again.
            p_world.refresh_model_constants();
            return true;
        case STIFFNESS:
            model->jnt_stiffness[id] = get_attribute(STIFFNESS);
            return true;
        case LIMITED:
        case RANGE_LOWER:
        case RANGE_UPPER: {
            // The same rule as add_to_spec: limited only with a usable range.
            const double lower = applies(RANGE_LOWER) ? double(get_attribute(RANGE_LOWER)) : 0.0;
            const double upper = applies(RANGE_UPPER) ? double(get_attribute(RANGE_UPPER)) : 0.0;
            const bool limited = applies(LIMITED) && bool(get_attribute(LIMITED)) && lower < upper;
            model->jnt_limited[id] = limited;
            model->jnt_range[2 * id] = lower;
            model->jnt_range[2 * id + 1] = upper;
            return true;
        }
    }
    // type, axis, ref, springref: these shape the model itself.
    return false;
}
