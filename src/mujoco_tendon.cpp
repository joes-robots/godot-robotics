#include "mujoco_tendon.h"
#include "mujoco_convert.h"
#include "mujoco_geom.h"
#include "mujoco_joint.h"
#include "mujoco_site.h"
#include "mujoco_world.h"

#include <godot_cpp/classes/cylinder_mesh.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/multi_mesh.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>

using namespace godot;

namespace {

const MujocoAttribute TENDON_ATTRIBUTES[] = {
    { "limited", Variant::BOOL, { 0 } },
    { "range", Variant::VECTOR2, { 0, 1 } },
    { "stiffness", Variant::FLOAT, { 0 } },
    { "damping", Variant::FLOAT, { 0 } },
    { "frictionloss", Variant::FLOAT, { 0 } },
    { "armature", Variant::FLOAT, { 0 } },
    { "spring_length", Variant::FLOAT, { -1 } }, // -1: the length in the scene pose
    { "actuator_force_limited", Variant::BOOL, { 0 } },
    { "actuator_force_range", Variant::VECTOR2, { -1, 1 } },
    // Limit behavior, with MuJoCo's defaults (as on joints).
    { "limit_margin", Variant::FLOAT, { 0 } },
    { "limit_solref_timeconst", Variant::FLOAT, { 0.02 } },
    { "limit_solref_dampratio", Variant::FLOAT, { 1.0 } },
    { "limit_solimp_dmin", Variant::FLOAT, { 0.9 } },
    { "limit_solimp_dmax", Variant::FLOAT, { 0.95 } },
    { "limit_solimp_width", Variant::FLOAT, { 0.001 } },
    { "limit_solimp_midpoint", Variant::FLOAT, { 0.5 } },
    { "limit_solimp_power", Variant::FLOAT, { 2.0 } },
    // Display, with MuJoCo's defaults.
    { "width", Variant::FLOAT, { 0.003 } },
    { "color", Variant::COLOR, { 0.5, 0.5, 0.5, 1.0 } },
};
const MujocoAttributeTable TENDON_TABLE = MUJOCO_ATTRIBUTE_TABLE(TENDON_ATTRIBUTES);

// MuJoCo marks a pulley in a spatial tendon's computed path with this object id.
constexpr int WRAP_OBJ_PULLEY = -2;

} // namespace

// --- MujocoTendon ---------------------------------------------------------------

const MujocoAttributeTable *MujocoTendon::get_attribute_table() const
{
    return &TENDON_TABLE;
}

void MujocoTendon::overrides_changed()
{
    notify_property_list_changed();
    changed();
}

bool MujocoTendon::get_path(std::vector<MujocoTendonWrap *> &r_wraps, bool &r_fixed, String &r_problem) const
{
    r_wraps.clear();
    r_fixed = false;
    if (!get_world()) {
        r_problem = "Not inside a MujocoWorld.";
        return false;
    }
    bool has_joints = false;
    bool has_others = false;
    for (int i = 0; i < get_child_count(); i++) {
        MujocoTendonWrap *wrap = Object::cast_to<MujocoTendonWrap>(get_child(i));
        if (!wrap) {
            continue;
        }
        Node *target = nullptr;
        String problem;
        const MujocoTendonWrap::Kind kind = wrap->get_kind(target, problem);
        if (kind == MujocoTendonWrap::KIND_INVALID) {
            r_problem = vformat("Path step \"%s\": %s", wrap->get_name(), problem);
            return false;
        }
        has_joints = has_joints || kind == MujocoTendonWrap::KIND_JOINT;
        has_others = has_others || kind != MujocoTendonWrap::KIND_JOINT;
        r_wraps.push_back(wrap);
    }
    if (r_wraps.empty()) {
        r_problem = "The path is empty: add MujocoTendonWrap children (joints for a fixed tendon; sites, geoms and pulleys for a spatial one).";
        return false;
    }
    if (has_joints && has_others) {
        r_problem = "The path mixes joints with sites, geoms or pulleys. A tendon is either fixed (joints only) or spatial (no joints).";
        return false;
    }
    r_fixed = has_joints;
    if (!r_fixed) {
        Node *first = nullptr;
        Node *last = nullptr;
        String unused;
        if (r_wraps.front()->get_kind(first, unused) != MujocoTendonWrap::KIND_SITE || r_wraps.back()->get_kind(last, unused) != MujocoTendonWrap::KIND_SITE) {
            r_problem = "A spatial tendon must start and end at a site.";
            return false;
        }
    }
    return true;
}

mjsBody *MujocoTendon::add_to_spec(mjSpec *p_spec, mjsBody *p_body, const Transform3D & /*p_transform*/, const String &p_name)
{
    std::vector<MujocoTendonWrap *> wraps;
    bool fixed = false;
    String problem;
    if (!get_path(wraps, fixed, problem)) {
        // Left out, like an actuator without a joint.
        if (!Engine::get_singleton()->is_editor_hint()) {
            WARN_PRINT(vformat("MuJoCo tendon '%s' is left out of the model: %s", p_name, problem));
        }
        return p_body;
    }
    const MujocoWorld *world = get_world();
    mjsTendon *tendon = mjs_addTendon(p_spec, get_spec_default(p_spec));
    mjs_setName(tendon->element, p_name.utf8().get_data());
    write_attributes(tendon);
    // Element names are their node paths relative to the world.
    auto name_of = [world](Node *p_node) { return String(world->get_path_to(p_node)).utf8(); };
    for (MujocoTendonWrap *wrap : wraps) {
        Node *target = nullptr;
        String unused;
        switch (wrap->get_kind(target, unused)) {
            case MujocoTendonWrap::KIND_JOINT:
                mjs_wrapJoint(tendon, name_of(target).get_data(), wrap->get_coef());
                break;
            case MujocoTendonWrap::KIND_SITE:
                mjs_wrapSite(tendon, name_of(target).get_data());
                break;
            case MujocoTendonWrap::KIND_GEOM: {
                Node *side = wrap->find_side_site(unused);
                mjs_wrapGeom(tendon, name_of(target).get_data(), side ? name_of(side).get_data() : "");
            } break;
            case MujocoTendonWrap::KIND_PULLEY:
                mjs_wrapPulley(tendon, wrap->get_divisor());
                break;
            case MujocoTendonWrap::KIND_INVALID:
                break;
        }
    }
    return p_body;
}

void MujocoTendon::write_attributes(mjsTendon *p_tendon) const
{
    const Vector2 range = get_range();
    p_tendon->limited = is_limited() ? mjLIMITED_TRUE : mjLIMITED_FALSE;
    p_tendon->range[0] = range.x;
    p_tendon->range[1] = range.y;
    p_tendon->stiffness[0] = get_stiffness();
    p_tendon->damping[0] = get_damping();
    p_tendon->frictionloss = get_frictionloss();
    p_tendon->armature = get_armature();
    p_tendon->springlength[0] = get_spring_length();
    p_tendon->springlength[1] = get_spring_length();
    const Vector2 actuator_force_range = get_actuator_force_range();
    p_tendon->actfrclimited = is_actuator_force_limited() ? mjLIMITED_TRUE : mjLIMITED_FALSE;
    p_tendon->actfrcrange[0] = actuator_force_range.x;
    p_tendon->actfrcrange[1] = actuator_force_range.y;
    p_tendon->margin = get_limit_margin();
    p_tendon->solref_limit[0] = get_limit_solref_timeconst();
    p_tendon->solref_limit[1] = get_limit_solref_dampratio();
    p_tendon->solimp_limit[0] = get_limit_solimp_dmin();
    p_tendon->solimp_limit[1] = get_limit_solimp_dmax();
    p_tendon->solimp_limit[2] = get_limit_solimp_width();
    p_tendon->solimp_limit[3] = get_limit_solimp_midpoint();
    p_tendon->solimp_limit[4] = get_limit_solimp_power();
    p_tendon->width = get_width();
    const Color color = get_color();
    p_tendon->rgba[0] = color.r;
    p_tendon->rgba[1] = color.g;
    p_tendon->rgba[2] = color.b;
    p_tendon->rgba[3] = color.a;
}

void MujocoTendon::write_class_defaults(mjsDefault *p_default)
{
    write_attributes(p_default->tendon);
    // MJCF doesn't allow a tendon's actuator force limits in a class (MuJoCo's
    // writer would still save them, and its parser then rejects the file), so the
    // class keeps MuJoCo's defaults; each tendon writes its own.
    p_default->tendon->actfrclimited = mjLIMITED_FALSE;
    p_default->tendon->actfrcrange[0] = 0.0;
    p_default->tendon->actfrcrange[1] = 0.0;
}

void MujocoTendon::inherited_changed()
{
    MujocoElement::inherited_changed();
    // Width and color may be inherited.
    update_material();
    update_drawing();
}

PackedStringArray MujocoTendon::_get_configuration_warnings() const
{
    PackedStringArray warnings = MujocoElement::_get_configuration_warnings();
    std::vector<MujocoTendonWrap *> wraps;
    bool fixed = false;
    String problem;
    if (get_world() && !get_path(wraps, fixed, problem)) {
        warnings.push_back(problem + " This tendon is left out of the model.");
    }
    if (is_limited() && get_range().x >= get_range().y) {
        warnings.push_back("Range minimum must be less than its maximum.");
    }
    return warnings;
}

void MujocoTendon::changed()
{
    update_configuration_warnings();
    update_material();
    request_compile();
}

void MujocoTendon::path_changed()
{
    update_configuration_warnings();
    request_compile();
}

double MujocoTendon::get_length()
{
    const int id = get_model_id(mjOBJ_TENDON);
    ERR_FAIL_COND_V_MSG(id < 0, 0.0, "This tendon is not part of a compiled MuJoCo model.");
    return get_world()->get_data()->ten_length[id];
}

double MujocoTendon::get_velocity()
{
    const int id = get_model_id(mjOBJ_TENDON);
    ERR_FAIL_COND_V_MSG(id < 0, 0.0, "This tendon is not part of a compiled MuJoCo model.");
    return get_world()->get_data()->ten_velocity[id];
}

void MujocoTendon::update_material()
{
    if (!path_instance) {
        return;
    }
    const Color color = get_color();
    Ref<StandardMaterial3D> material;
    material.instantiate();
    material->set_albedo(color);
    if (color.a < 1.0f) {
        material->set_transparency(BaseMaterial3D::TRANSPARENCY_ALPHA);
    }
    path_instance->set_material_override(material);
}

void MujocoTendon::update_drawing()
{
    if (!path_instance) {
        return;
    }
    const Ref<MultiMesh> multimesh = path_instance->get_multimesh();
    MujocoWorld *world = get_world();
    const int id = get_model_id(mjOBJ_TENDON);
    const mjModel *m = world ? world->get_model() : nullptr;
    const mjData *d = world ? world->get_data() : nullptr;
    if (id < 0 || !m || !d) {
        multimesh->set_instance_count(0);
        return;
    }
    // MuJoCo's computed path, in the world's frame: segments between consecutive
    // points, except across a pulley. (Fixed tendons have no path.)
    std::vector<Transform3D> segments;
    const double radius = get_width();
    const int start = d->ten_wrapadr[id];
    const int count = d->ten_wrapnum[id];
    for (int k = start; k + 1 < start + count; k++) {
        if (d->wrap_obj[k] == WRAP_OBJ_PULLEY || d->wrap_obj[k + 1] == WRAP_OBJ_PULLEY) {
            continue;
        }
        const Vector3 a = mujoco_convert::to_godot(d->wrap_xpos + 3 * k);
        const Vector3 b = mujoco_convert::to_godot(d->wrap_xpos + 3 * (k + 1));
        const Vector3 along = b - a;
        if (along.length() < 1e-9) {
            continue;
        }
        // A unit cylinder along Y, stretched from a to b and scaled to the radius.
        const Vector3 y = along;
        const Vector3 x = y.cross(std::abs(y.normalized().y) < 0.9 ? Vector3(0, 1, 0) : Vector3(1, 0, 0)).normalized() * radius;
        const Vector3 z = x.cross(y).normalized() * radius;
        segments.push_back(Transform3D(Basis(x, y, z), (a + b) * 0.5));
    }
    multimesh->set_instance_count(static_cast<int32_t>(segments.size()));
    for (size_t i = 0; i < segments.size(); i++) {
        multimesh->set_instance_transform(static_cast<int32_t>(i), segments[i]);
    }
    if (world->is_inside_tree() && path_instance->is_inside_tree()) {
        path_instance->set_global_transform(world->get_global_transform());
    }
}

void MujocoTendon::_notification(int p_what)
{
    switch (p_what) {
        case NOTIFICATION_ENTER_TREE:
            if (!path_instance) {
                // Drawn in the world's frame, so independent of this node's transform.
                path_instance = memnew(MultiMeshInstance3D);
                path_instance->set_as_top_level(true);
                Ref<CylinderMesh> cylinder;
                cylinder.instantiate();
                cylinder->set_top_radius(1.0);
                cylinder->set_bottom_radius(1.0);
                cylinder->set_height(1.0);
                cylinder->set_radial_segments(8);
                cylinder->set_rings(1);
                Ref<MultiMesh> multimesh;
                multimesh.instantiate();
                multimesh->set_transform_format(MultiMesh::TRANSFORM_3D);
                multimesh->set_mesh(cylinder);
                path_instance->set_multimesh(multimesh);
                add_child(path_instance, false, INTERNAL_MODE_FRONT);
                update_material();
            }
            break;
        case NOTIFICATION_CHILD_ORDER_CHANGED:
            // The path is the order of the wrap children.
            path_changed();
            break;
    }
}

void MujocoTendon::_validate_property(PropertyInfo &p_property) const
{
    if ((p_property.name == StringName("range") && !is_limited()) ||
            (p_property.name == StringName("actuator_force_range") && !is_actuator_force_limited())) {
        p_property.usage = PROPERTY_USAGE_NONE; // hidden; the value is stored in overrides
    }
}

void MujocoTendon::_bind_methods()
{
    ClassDB::bind_method(D_METHOD("set_limited", "limited"), &MujocoTendon::set_limited);
    ClassDB::bind_method(D_METHOD("is_limited"), &MujocoTendon::is_limited);
    ClassDB::bind_method(D_METHOD("set_range", "range"), &MujocoTendon::set_range);
    ClassDB::bind_method(D_METHOD("get_range"), &MujocoTendon::get_range);
    ClassDB::bind_method(D_METHOD("set_color", "color"), &MujocoTendon::set_color);
    ClassDB::bind_method(D_METHOD("get_color"), &MujocoTendon::get_color);
    ClassDB::bind_method(D_METHOD("set_actuator_force_limited", "limited"), &MujocoTendon::set_actuator_force_limited);
    ClassDB::bind_method(D_METHOD("is_actuator_force_limited"), &MujocoTendon::is_actuator_force_limited);
    ClassDB::bind_method(D_METHOD("set_actuator_force_range", "range"), &MujocoTendon::set_actuator_force_range);
    ClassDB::bind_method(D_METHOD("get_actuator_force_range"), &MujocoTendon::get_actuator_force_range);
#define TENDON_BIND_NUMBER(m_name) \
    ClassDB::bind_method(D_METHOD("set_" #m_name, "value"), &MujocoTendon::set_##m_name); \
    ClassDB::bind_method(D_METHOD("get_" #m_name), &MujocoTendon::get_##m_name);
    TENDON_BIND_NUMBER(stiffness)
    TENDON_BIND_NUMBER(damping)
    TENDON_BIND_NUMBER(frictionloss)
    TENDON_BIND_NUMBER(armature)
    TENDON_BIND_NUMBER(spring_length)
    TENDON_BIND_NUMBER(limit_margin)
    TENDON_BIND_NUMBER(limit_solref_timeconst)
    TENDON_BIND_NUMBER(limit_solref_dampratio)
    TENDON_BIND_NUMBER(limit_solimp_dmin)
    TENDON_BIND_NUMBER(limit_solimp_dmax)
    TENDON_BIND_NUMBER(limit_solimp_width)
    TENDON_BIND_NUMBER(limit_solimp_midpoint)
    TENDON_BIND_NUMBER(limit_solimp_power)
    TENDON_BIND_NUMBER(width)
#undef TENDON_BIND_NUMBER
    ClassDB::bind_method(D_METHOD("get_length"), &MujocoTendon::get_length);
    ClassDB::bind_method(D_METHOD("get_velocity"), &MujocoTendon::get_velocity);

    // Attribute properties are stored in "overrides", so they are editor-only here.
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "limited", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR), "set_limited", "is_limited");
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "range", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR), "set_range", "get_range");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "stiffness", PROPERTY_HINT_RANGE, "0,1000,0.001,or_greater", PROPERTY_USAGE_EDITOR), "set_stiffness", "get_stiffness");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "damping", PROPERTY_HINT_RANGE, "0,100,0.001,or_greater", PROPERTY_USAGE_EDITOR), "set_damping", "get_damping");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "frictionloss", PROPERTY_HINT_RANGE, "0,100,0.001,or_greater", PROPERTY_USAGE_EDITOR), "set_frictionloss", "get_frictionloss");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "armature", PROPERTY_HINT_RANGE, "0,10,0.0001,or_greater", PROPERTY_USAGE_EDITOR), "set_armature", "get_armature");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "spring_length", PROPERTY_HINT_RANGE, "-1,10,0.0001,or_greater", PROPERTY_USAGE_EDITOR), "set_spring_length", "get_spring_length");

    ADD_GROUP("Actuator Force", "actuator_force_");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "actuator_force_limited", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR), "set_actuator_force_limited", "is_actuator_force_limited");
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "actuator_force_range", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR), "set_actuator_force_range", "get_actuator_force_range");

    ADD_GROUP("Limit", "limit_");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "limit_margin", PROPERTY_HINT_RANGE, "0,1,0.0001,or_greater", PROPERTY_USAGE_EDITOR), "set_limit_margin", "get_limit_margin");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "limit_solref_timeconst", PROPERTY_HINT_RANGE, "0.0001,1,0.0001,or_greater,or_less,suffix:s", PROPERTY_USAGE_EDITOR), "set_limit_solref_timeconst", "get_limit_solref_timeconst");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "limit_solref_dampratio", PROPERTY_HINT_RANGE, "0,2,0.001,or_greater,or_less", PROPERTY_USAGE_EDITOR), "set_limit_solref_dampratio", "get_limit_solref_dampratio");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "limit_solimp_dmin", PROPERTY_HINT_RANGE, "0.0001,0.9999,0.0001", PROPERTY_USAGE_EDITOR), "set_limit_solimp_dmin", "get_limit_solimp_dmin");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "limit_solimp_dmax", PROPERTY_HINT_RANGE, "0.0001,0.9999,0.0001", PROPERTY_USAGE_EDITOR), "set_limit_solimp_dmax", "get_limit_solimp_dmax");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "limit_solimp_width", PROPERTY_HINT_RANGE, "0,1,0.0001,or_greater", PROPERTY_USAGE_EDITOR), "set_limit_solimp_width", "get_limit_solimp_width");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "limit_solimp_midpoint", PROPERTY_HINT_RANGE, "0,1,0.001", PROPERTY_USAGE_EDITOR), "set_limit_solimp_midpoint", "get_limit_solimp_midpoint");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "limit_solimp_power", PROPERTY_HINT_RANGE, "1,10,0.01,or_greater", PROPERTY_USAGE_EDITOR), "set_limit_solimp_power", "get_limit_solimp_power");

    ADD_GROUP("Display", "");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "width", PROPERTY_HINT_RANGE, "0.0001,0.1,0.0001,or_greater,suffix:m", PROPERTY_USAGE_EDITOR), "set_width", "get_width");
    ADD_PROPERTY(PropertyInfo(Variant::COLOR, "color", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR), "set_color", "get_color");
}

void MujocoTendon::set_limited(bool p_limited)
{
    set_attribute("limited", p_limited);
    notify_property_list_changed();
    changed();
}

bool MujocoTendon::is_limited() const { return get_attribute("limited"); }

void MujocoTendon::set_range(const Vector2 &p_range)
{
    set_attribute("range", p_range);
    changed();
}

Vector2 MujocoTendon::get_range() const { return get_attribute("range"); }

void MujocoTendon::set_color(const Color &p_color)
{
    set_attribute("color", p_color);
    changed();
}

Color MujocoTendon::get_color() const { return get_attribute("color"); }

// The numeric attributes; MuJoCo's compiler checks their values.
#define TENDON_NUMBER_ATTRIBUTE(m_name) \
    void MujocoTendon::set_##m_name(double p_value) \
    { \
        set_attribute(#m_name, p_value); \
        changed(); \
    } \
    double MujocoTendon::get_##m_name() const { return get_attribute(#m_name); }

TENDON_NUMBER_ATTRIBUTE(stiffness)
TENDON_NUMBER_ATTRIBUTE(damping)
TENDON_NUMBER_ATTRIBUTE(frictionloss)
TENDON_NUMBER_ATTRIBUTE(armature)
TENDON_NUMBER_ATTRIBUTE(spring_length)
TENDON_NUMBER_ATTRIBUTE(limit_margin)
TENDON_NUMBER_ATTRIBUTE(limit_solref_timeconst)
TENDON_NUMBER_ATTRIBUTE(limit_solref_dampratio)
TENDON_NUMBER_ATTRIBUTE(limit_solimp_dmin)
TENDON_NUMBER_ATTRIBUTE(limit_solimp_dmax)
TENDON_NUMBER_ATTRIBUTE(limit_solimp_width)
TENDON_NUMBER_ATTRIBUTE(limit_solimp_midpoint)
TENDON_NUMBER_ATTRIBUTE(limit_solimp_power)
TENDON_NUMBER_ATTRIBUTE(width)

#undef TENDON_NUMBER_ATTRIBUTE

// --- MujocoTendonWrap -----------------------------------------------------------

MujocoTendon *MujocoTendonWrap::get_tendon() const
{
    return Object::cast_to<MujocoTendon>(get_parent());
}

MujocoTendonWrap::Kind MujocoTendonWrap::get_kind(Node *&r_target, String &r_problem) const
{
    r_target = nullptr;
    const MujocoTendon *tendon = get_tendon();
    if (!tendon) {
        r_problem = "Must be a child of a MujocoTendon.";
        return KIND_INVALID;
    }
    if (target.is_empty()) {
        return KIND_PULLEY;
    }
    Node *node = get_node_or_null(target);
    const MujocoElement *element = Object::cast_to<MujocoElement>(node);
    if (!element) {
        r_problem = "\"target\" must point to a hinge or slide MujocoJoint, a MujocoSite or a MujocoGeom (or be empty for a pulley).";
        return KIND_INVALID;
    }
    if (element->get_world() != tendon->get_world()) {
        r_problem = "The target belongs to a different MujocoWorld.";
        return KIND_INVALID;
    }
    r_target = node;
    if (const MujocoJoint *joint = Object::cast_to<MujocoJoint>(node)) {
        if (joint->get_type() != MujocoJoint::TYPE_HINGE && joint->get_type() != MujocoJoint::TYPE_SLIDE) {
            r_problem = "Only hinge and slide joints can be part of a tendon.";
            return KIND_INVALID;
        }
        return KIND_JOINT;
    }
    if (Object::cast_to<MujocoSite>(node)) {
        return KIND_SITE;
    }
    if (const MujocoGeom *geom = Object::cast_to<MujocoGeom>(node)) {
        if (geom->get_shape() != MujocoGeom::SHAPE_SPHERE && geom->get_shape() != MujocoGeom::SHAPE_CYLINDER) {
            r_problem = "A tendon can only wrap around a sphere or a cylinder geom.";
            return KIND_INVALID;
        }
        String side_problem;
        find_side_site(side_problem);
        if (!side_problem.is_empty()) {
            r_problem = side_problem;
            return KIND_INVALID;
        }
        return KIND_GEOM;
    }
    r_problem = "\"target\" must point to a hinge or slide MujocoJoint, a MujocoSite or a MujocoGeom (or be empty for a pulley).";
    return KIND_INVALID;
}

Node *MujocoTendonWrap::find_side_site(String &r_problem) const
{
    if (side_site.is_empty()) {
        return nullptr;
    }
    Node *node = get_node_or_null(side_site);
    if (!Object::cast_to<MujocoSite>(node)) {
        r_problem = "\"side_site\" must point to a MujocoSite (or be empty).";
        return nullptr;
    }
    return node;
}

PackedStringArray MujocoTendonWrap::_get_configuration_warnings() const
{
    PackedStringArray warnings = Node::_get_configuration_warnings();
    Node *node = nullptr;
    String problem;
    if (get_kind(node, problem) == KIND_INVALID) {
        warnings.push_back(problem);
    }
    return warnings;
}

void MujocoTendonWrap::changed()
{
    update_configuration_warnings();
    notify_property_list_changed();
    if (MujocoTendon *tendon = get_tendon()) {
        tendon->path_changed();
    }
}

void MujocoTendonWrap::_notification(int p_what)
{
    switch (p_what) {
        case NOTIFICATION_ENTER_TREE:
        case NOTIFICATION_EXIT_TREE:
            // The parent chain is still intact on exit, so the tendon can be found.
            changed();
            break;
    }
}

void MujocoTendonWrap::_validate_property(PropertyInfo &p_property) const
{
    // Only the settings of this kind of step.
    Node *node = nullptr;
    String problem;
    const Kind kind = get_kind(node, problem);
    const StringName name = p_property.name;
    if ((name == StringName("coef") && kind != KIND_JOINT) ||
            (name == StringName("side_site") && kind != KIND_GEOM) ||
            (name == StringName("divisor") && kind != KIND_PULLEY)) {
        p_property.usage = PROPERTY_USAGE_STORAGE;
    }
}

void MujocoTendonWrap::_bind_methods()
{
    BIND_ENUM_CONSTANT(KIND_JOINT);
    BIND_ENUM_CONSTANT(KIND_SITE);
    BIND_ENUM_CONSTANT(KIND_GEOM);
    BIND_ENUM_CONSTANT(KIND_PULLEY);
    BIND_ENUM_CONSTANT(KIND_INVALID);

    ClassDB::bind_method(D_METHOD("set_target", "target"), &MujocoTendonWrap::set_target);
    ClassDB::bind_method(D_METHOD("get_target"), &MujocoTendonWrap::get_target);
    ClassDB::bind_method(D_METHOD("set_coef", "coef"), &MujocoTendonWrap::set_coef);
    ClassDB::bind_method(D_METHOD("get_coef"), &MujocoTendonWrap::get_coef);
    ClassDB::bind_method(D_METHOD("set_side_site", "site"), &MujocoTendonWrap::set_side_site);
    ClassDB::bind_method(D_METHOD("get_side_site"), &MujocoTendonWrap::get_side_site);
    ClassDB::bind_method(D_METHOD("set_divisor", "divisor"), &MujocoTendonWrap::set_divisor);
    ClassDB::bind_method(D_METHOD("get_divisor"), &MujocoTendonWrap::get_divisor);

    ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "target", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "MujocoJoint,MujocoSite,MujocoGeom"), "set_target", "get_target");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "coef", PROPERTY_HINT_RANGE, "-10,10,0.001,or_less,or_greater"), "set_coef", "get_coef");
    ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "side_site", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "MujocoSite"), "set_side_site", "get_side_site");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "divisor", PROPERTY_HINT_RANGE, "0.001,10,0.001,or_greater"), "set_divisor", "get_divisor");
}

void MujocoTendonWrap::set_target(const NodePath &p_target)
{
    target = p_target;
    changed();
}

NodePath MujocoTendonWrap::get_target() const { return target; }

void MujocoTendonWrap::set_coef(double p_coef)
{
    coef = p_coef;
    changed();
}

double MujocoTendonWrap::get_coef() const { return coef; }

void MujocoTendonWrap::set_side_site(const NodePath &p_site)
{
    side_site = p_site;
    changed();
}

NodePath MujocoTendonWrap::get_side_site() const { return side_site; }

void MujocoTendonWrap::set_divisor(double p_divisor)
{
    ERR_FAIL_COND_MSG(p_divisor <= 0.0, "Divisor must be positive.");
    divisor = p_divisor;
    changed();
}

double MujocoTendonWrap::get_divisor() const { return divisor; }

void MujocoTendon::set_actuator_force_limited(bool p_limited)
{
    set_attribute("actuator_force_limited", p_limited);
    notify_property_list_changed();
    changed();
}

bool MujocoTendon::is_actuator_force_limited() const { return get_attribute("actuator_force_limited"); }

void MujocoTendon::set_actuator_force_range(const Vector2 &p_range)
{
    set_attribute("actuator_force_range", p_range);
    changed();
}

Vector2 MujocoTendon::get_actuator_force_range() const { return get_attribute("actuator_force_range"); }
