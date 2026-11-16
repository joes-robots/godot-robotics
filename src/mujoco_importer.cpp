#include "mujoco_importer.h"

#include <cmath>
#include <set>

#include <godot_cpp/classes/project_settings.hpp>
#include <godot_cpp/variant/dictionary.hpp>

#include "mujoco_body.h"
#include "mujoco_convert.h"
#include "mujoco_geom.h"
#include "mujoco_joint.h"
#include "mujoco_options.h"
#include "mujoco_verify.h"
#include "mujoco_world.h"

using namespace godot;

namespace {

// The name an element has in the MJCF file; empty if it has none.
String element_name(mjsElement *p_element)
{
    return String::utf8(mjs_getString(mjs_getName(p_element)));
}

// A name for reports: the element's own, or a description if it has none.
String describe(mjsElement *p_element, const char *p_kind)
{
    const String name = element_name(p_element);
    return name.is_empty() ? String("(unnamed ") + p_kind + String(")") : name;
}

// A node's path from the world, as MujocoElement would compute it once in the tree.
String path_from(const Node *p_world, const Node *p_node)
{
    String path = p_node->get_name();
    for (const Node *node = p_node->get_parent(); node && node != p_world; node = node->get_parent()) {
        path = String(node->get_name()) + String("/") + path;
    }
    return path;
}

bool differs(const mjtNum *p_a, const mjtNum *p_b, int p_count)
{
    for (int i = 0; i < p_count; i++) {
        if (p_a[i] != p_b[i]) {
            return true;
        }
    }
    return false;
}

} // namespace

void MujocoImporter::_bind_methods()
{
    ClassDB::bind_method(D_METHOD("import_file", "path"), &MujocoImporter::import_file);
    ClassDB::bind_method(D_METHOD("import_string", "xml"), &MujocoImporter::import_string);
    ClassDB::bind_method(D_METHOD("get_error"), &MujocoImporter::get_error);
    ClassDB::bind_method(D_METHOD("get_report"), &MujocoImporter::get_report);
    ClassDB::bind_method(D_METHOD("get_report_text"), &MujocoImporter::get_report_text);
    ClassDB::bind_method(D_METHOD("verify_file", "world", "path"), &MujocoImporter::verify_file);
    ClassDB::bind_method(D_METHOD("verify_string", "world", "xml"), &MujocoImporter::verify_string);
    ClassDB::bind_static_method("MujocoImporter", D_METHOD("differences_to_text", "differences"), &MujocoImporter::differences_to_text);
}

String MujocoImporter::get_error() const
{
    return error;
}

Array MujocoImporter::get_report() const
{
    return report.duplicate(true);
}

String MujocoImporter::get_report_text() const
{
    String text;
    for (int i = 0; i < report.size(); i++) {
        const Dictionary entry = report[i];
        const String element = entry["element"];
        if (!element.is_empty()) {
            text += element + String(": ");
        }
        text += String(entry["message"]) + String("\n");
    }
    return text;
}

void MujocoImporter::add_report(const String &p_kind, const String &p_element, const String &p_message)
{
    Dictionary entry;
    entry["kind"] = p_kind;
    entry["element"] = p_element;
    entry["message"] = p_message;
    report.push_back(entry);
}

MujocoWorld *MujocoImporter::import_file(const String &p_path)
{
    error = String();
    report.clear();

    // MuJoCo opens the file itself (so that files it includes are found beside
    // it), and needs a real path, not a res:// one.
    const String path = ProjectSettings::get_singleton()->globalize_path(p_path);
    char parse_error[1024] = "";
    mjSpec *spec = mj_parseXML(path.utf8().get_data(), nullptr, parse_error, sizeof(parse_error));
    if (!spec) {
        error = String("Could not read '") + p_path + String("': ") + String::utf8(parse_error);
        return nullptr;
    }
    return build(spec, p_path.get_file().get_basename());
}

MujocoWorld *MujocoImporter::import_string(const String &p_xml)
{
    error = String();
    report.clear();

    char parse_error[1024] = "";
    mjSpec *spec = mj_parseXMLString(p_xml.utf8().get_data(), nullptr, parse_error, sizeof(parse_error));
    if (!spec) {
        error = String("Could not read the MJCF text: ") + String::utf8(parse_error);
        return nullptr;
    }
    return build(spec, "MujocoWorld");
}

// Takes ownership of p_spec.
MujocoWorld *MujocoImporter::build(mjSpec *p_spec, const String &p_fallback_name)
{
    mjModel *model = mj_compile(p_spec, nullptr);
    if (!model) {
        error = String("The MJCF model does not compile: ") + String::utf8(mjs_getError(p_spec));
        mj_deleteSpec(p_spec);
        return nullptr;
    }

    MujocoWorld *world = memnew(MujocoWorld);
    const String model_name = String::utf8(mjs_getString(p_spec->modelname));
    // "MuJoCo Model" is what MuJoCo calls a model that doesn't name itself.
    const bool is_named = !model_name.is_empty() && model_name != String("MuJoCo Model");
    world->set_name((is_named ? model_name : p_fallback_name).validate_node_name());

    import_options(world, model);
    import_children(world, world, mjs_findBody(p_spec, "world"), model);
    report_other_elements(p_spec);

    mj_deleteModel(model);
    mj_deleteSpec(p_spec);
    return world;
}

// ---- Options ----

void MujocoImporter::import_options(MujocoWorld *p_world, const mjModel *p_model)
{
    // Every option the world exposes, set where the file changes it.
    List<PropertyInfo> properties;
    mujoco_options::get_property_list(&properties);
    for (const PropertyInfo &property : properties) {
        Variant value, default_value;
        if (!mujoco_options::get(p_model->opt, property.name, value)) {
            continue; // the inspector group heading
        }
        mujoco_options::get_default(property.name, default_value);
        if (value != default_value) {
            p_world->set(property.name, value);
        }
    }

    // Options the world has no property for.
    mjOption defaults;
    mj_defaultOption(&defaults);
    const mjOption &option = p_model->opt;
    if (option.disableflags != defaults.disableflags || option.enableflags != defaults.enableflags) {
        add_report("skipped_attribute", "", "The option flags (<flag>) are not supported; MuJoCo's defaults are used.");
    }
    if (differs(option.o_solref, defaults.o_solref, mjNREF) || differs(option.o_solimp, defaults.o_solimp, mjNIMP)
            || differs(option.o_friction, defaults.o_friction, 5)) {
        add_report("skipped_attribute", "", "The contact override options (o_solref, o_solimp, o_friction) are not supported.");
    }
}

// ---- The body tree ----

void MujocoImporter::add_node(Node *p_parent, MujocoWorld *p_world, MujocoElement *p_node, mjsElement *p_element, const String &p_default_name)
{
    const String name = element_name(p_element);
    // Node names can't contain a few characters that MJCF names can.
    p_node->set_name(name.is_empty() ? p_default_name : name.validate_node_name());
    // `true` makes Godot rename the node if a sibling already has its name.
    p_parent->add_child(p_node, true);
    // Owned by the world, so that saving the world as a scene includes it.
    p_node->set_owner(p_world);

    // The compiled name would be the node's path. Where that isn't the name
    // the file gave it, keep the file's.
    if (!name.is_empty() && path_from(p_world, p_node) != name) {
        p_node->set_mujoco_name_override(name);
    }
}

void MujocoImporter::set_if_different(MujocoElement *p_node, const String &p_attribute, const Variant &p_value)
{
    // Before anything is set, reading an attribute gives the value it would
    // have anyway. Only a different value needs storing.
    if (p_node->get(p_attribute) != p_value) {
        p_node->set(p_attribute, p_value);
    }
}

// Imports what is directly inside p_body: its joints, its geoms, then its
// child bodies. p_parent is the node standing for p_body.
void MujocoImporter::import_children(Node *p_parent, MujocoWorld *p_world, mjsBody *p_body, const mjModel *p_model)
{
    // The 0 means "direct children only".
    for (mjsElement *e = mjs_firstChild(p_body, mjOBJ_JOINT, 0); e; e = mjs_nextChild(p_body, e, 0)) {
        import_joint(p_parent, p_world, mjs_asJoint(e), p_model);
    }
    for (mjsElement *e = mjs_firstChild(p_body, mjOBJ_GEOM, 0); e; e = mjs_nextChild(p_body, e, 0)) {
        import_geom(p_parent, p_world, mjs_asGeom(e), p_model);
    }
    for (mjsElement *e = mjs_firstChild(p_body, mjOBJ_BODY, 0); e; e = mjs_nextChild(p_body, e, 0)) {
        import_body(p_parent, p_world, mjs_asBody(e), p_model);
    }
}

void MujocoImporter::import_body(Node *p_parent, MujocoWorld *p_world, mjsBody *p_body, const mjModel *p_model)
{
    // Every element of the spec knows which entry of the compiled model it became.
    const int id = mjs_getId(p_body->element);
    const String name = describe(p_body->element, "body");
    if (id < 0) {
        add_report("skipped_element", name, "This body is not in the compiled model (MuJoCo merged or removed it), so it was skipped with everything in it.");
        return;
    }

    MujocoBody *body = memnew(MujocoBody);
    add_node(p_parent, p_world, body, p_body->element, "Body");
    // The body's pose relative to its parent body.
    body->set_transform(mujoco_convert::to_godot(&p_model->body_pos[3 * id], &p_model->body_quat[4 * id]));
    set_if_different(body, "mocap", p_model->body_mocapid[id] >= 0);

    if (p_body->explicitinertial) {
        add_report("skipped_attribute", name, "The body gives its mass and inertia explicitly (<inertial>), which is not supported; they are computed from its geoms instead.");
    }
    if (p_body->gravcomp != 0) {
        add_report("skipped_attribute", name, "Gravity compensation (gravcomp) is not supported.");
    }

    import_children(body, p_world, p_body, p_model);
}

void MujocoImporter::import_geom(Node *p_parent, MujocoWorld *p_world, mjsGeom *p_geom, const mjModel *p_model)
{
    const int id = mjs_getId(p_geom->element);
    const String name = describe(p_geom->element, "geom");
    if (id < 0) {
        add_report("skipped_element", name, "This geom is not in the compiled model, so it was skipped.");
        return;
    }

    const int shape = p_model->geom_type[id];
    const bool supported = shape == mjGEOM_PLANE || shape == mjGEOM_SPHERE || shape == mjGEOM_CAPSULE
            || shape == mjGEOM_CYLINDER || shape == mjGEOM_BOX;
    if (!supported) {
        const char *shape_name = shape == mjGEOM_MESH ? "mesh" : shape == mjGEOM_ELLIPSOID ? "ellipsoid"
                : shape == mjGEOM_HFIELD ? "height field" : "this kind of";
        add_report("skipped_element", name, String("Geoms of ") + shape_name + String(" shape are not supported; the geom was skipped. Its body is missing its mass and collision."));
        return;
    }

    MujocoGeom *geom = memnew(MujocoGeom);
    add_node(p_parent, p_world, geom, p_geom->element, "Geom");
    geom->set_transform(mujoco_convert::to_godot(&p_model->geom_pos[3 * id], &p_model->geom_quat[4 * id]));
    set_if_different(geom, "shape", shape);

    // The compiled size, laid out as MujocoGeom's size attributes (see there).
    const mjtNum *size = &p_model->geom_size[3 * id];
    switch (shape) {
        case mjGEOM_SPHERE:
            set_if_different(geom, "radius", size[0]);
            break;
        case mjGEOM_CAPSULE:
        case mjGEOM_CYLINDER:
            set_if_different(geom, "radius", size[0]);
            set_if_different(geom, "half_length", size[1]);
            break;
        case mjGEOM_BOX:
            set_if_different(geom, "half_extents", Vector3(size[0], size[2], size[1]));
            break;
        case mjGEOM_PLANE:
            set_if_different(geom, "plane_half_size", Vector2(size[0], size[1]));
            set_if_different(geom, "grid_spacing", size[2]);
            break;
    }

    const float *rgba = &p_model->geom_rgba[4 * id];
    set_if_different(geom, "color", Color(rgba[0], rgba[1], rgba[2], rgba[3]));
    set_if_different(geom, "friction_sliding", p_model->geom_friction[3 * id]);
    set_if_different(geom, "friction_torsional", p_model->geom_friction[3 * id + 1]);
    set_if_different(geom, "friction_rolling", p_model->geom_friction[3 * id + 2]);
    set_if_different(geom, "contype", p_model->geom_contype[id]);
    set_if_different(geom, "conaffinity", p_model->geom_conaffinity[id]);
    // Mass and density are inputs to the compiler and are not kept per geom in
    // the compiled model, so these two come from the spec.
    set_if_different(geom, "density", p_geom->density);
    if (!std::isnan(p_geom->mass)) {
        set_if_different(geom, "mass", p_geom->mass);
    }

    // Attributes MujocoGeom doesn't have.
    mjsGeom defaults;
    mjs_defaultGeom(&defaults);
    PackedStringArray unsupported;
    if (p_geom->condim != defaults.condim) unsupported.push_back("condim");
    if (p_geom->priority != defaults.priority) unsupported.push_back("priority");
    if (p_geom->solmix != defaults.solmix) unsupported.push_back("solmix");
    if (differs(p_geom->solref, defaults.solref, mjNREF)) unsupported.push_back("solref");
    if (differs(p_geom->solimp, defaults.solimp, mjNIMP)) unsupported.push_back("solimp");
    if (p_geom->margin != defaults.margin) unsupported.push_back("margin");
    if (p_geom->gap != defaults.gap) unsupported.push_back("gap");
    if (p_geom->group != defaults.group) unsupported.push_back("group");
    if (p_geom->fluid_ellipsoid != defaults.fluid_ellipsoid) unsupported.push_back("fluidshape");
    if (!String::utf8(mjs_getString(p_geom->material)).is_empty()) unsupported.push_back("material");
    if (!unsupported.is_empty()) {
        add_report("skipped_attribute", name, String("Not supported, so MuJoCo's defaults are used: ") + String(", ").join(unsupported) + String("."));
    }
}

void MujocoImporter::import_joint(Node *p_parent, MujocoWorld *p_world, mjsJoint *p_joint, const mjModel *p_model)
{
    const int id = mjs_getId(p_joint->element);
    const String name = describe(p_joint->element, "joint");
    if (id < 0) {
        add_report("skipped_element", name, "This joint is not in the compiled model, so it was skipped.");
        return;
    }

    MujocoJoint *joint = memnew(MujocoJoint);
    add_node(p_parent, p_world, joint, p_joint->element, "Joint");
    const int type = p_model->jnt_type[id];
    set_if_different(joint, "type", type);
    // The anchor. The node is left unrotated, so the axis attribute is the
    // joint's axis in its body, as in the file.
    joint->set_position(mujoco_convert::to_godot(&p_model->jnt_pos[3 * id]));
    if (type == mjJNT_HINGE || type == mjJNT_SLIDE) {
        set_if_different(joint, "axis", mujoco_convert::to_godot(&p_model->jnt_axis[3 * id]));
    }

    // Limits, already in radians and with "auto" resolved to yes or no.
    if (type != mjJNT_FREE && p_model->jnt_limited[id]) {
        joint->set("limited", true);
        if (type != mjJNT_BALL) {
            set_if_different(joint, "range_lower", p_model->jnt_range[2 * id]);
        }
        set_if_different(joint, "range_upper", p_model->jnt_range[2 * id + 1]);
    }

    // Per-joint values live in jnt_ arrays; per-degree-of-freedom ones in dof_
    // arrays, starting at the joint's first degree of freedom.
    const int qpos_address = p_model->jnt_qposadr[id];
    const int dof = p_model->jnt_dofadr[id];
    if (type == mjJNT_HINGE || type == mjJNT_SLIDE) {
        // The reference value is the joint's position in the model's initial pose.
        set_if_different(joint, "ref", p_model->qpos0[qpos_address]);
        set_if_different(joint, "springref", p_model->qpos_spring[qpos_address]);
    }
    set_if_different(joint, "stiffness", p_model->jnt_stiffness[id]);
    set_if_different(joint, "damping", p_model->dof_damping[dof]);
    set_if_different(joint, "armature", p_model->dof_armature[dof]);
    set_if_different(joint, "frictionloss", p_model->dof_frictionloss[dof]);

    // Attributes MujocoJoint doesn't have.
    mjsJoint defaults;
    mjs_defaultJoint(&defaults);
    PackedStringArray unsupported;
    if (differs(&p_joint->stiffness[1], &defaults.stiffness[1], mjNPOLY)) unsupported.push_back("higher-order stiffness");
    if (differs(&p_joint->damping[1], &defaults.damping[1], mjNPOLY)) unsupported.push_back("higher-order damping");
    if (p_joint->margin != defaults.margin) unsupported.push_back("margin");
    if (differs(p_joint->solref_limit, defaults.solref_limit, mjNREF)) unsupported.push_back("solreflimit");
    if (differs(p_joint->solimp_limit, defaults.solimp_limit, mjNIMP)) unsupported.push_back("solimplimit");
    if (differs(p_joint->solref_friction, defaults.solref_friction, mjNREF)) unsupported.push_back("solreffriction");
    if (differs(p_joint->solimp_friction, defaults.solimp_friction, mjNIMP)) unsupported.push_back("solimpfriction");
    if (p_joint->actfrcrange[0] != 0 || p_joint->actfrcrange[1] != 0) unsupported.push_back("actuatorfrcrange");
    if (p_joint->group != defaults.group) unsupported.push_back("group");
    if (!unsupported.is_empty()) {
        add_report("skipped_attribute", name, String("Not supported, so MuJoCo's defaults are used: ") + String(", ").join(unsupported) + String("."));
    }
}

// ---- Everything else in the file ----

void MujocoImporter::report_other_elements(mjSpec *p_spec)
{
    // Kinds of element there is no node for yet.
    struct Kind {
        mjtObj type;
        const char *singular;
        const char *plural;
    };
    const Kind kinds[] = {
        { mjOBJ_SITE, "site", "sites" },
        { mjOBJ_CAMERA, "camera", "cameras" },
        { mjOBJ_LIGHT, "light", "lights" },
        { mjOBJ_MESH, "mesh", "meshes" },
        { mjOBJ_HFIELD, "height field", "height fields" },
        { mjOBJ_TEXTURE, "texture", "textures" },
        { mjOBJ_MATERIAL, "material", "materials" },
        { mjOBJ_PAIR, "contact pair", "contact pairs" },
        { mjOBJ_EXCLUDE, "contact exclusion", "contact exclusions" },
        { mjOBJ_EQUALITY, "equality constraint", "equality constraints" },
        { mjOBJ_TENDON, "tendon", "tendons" },
        { mjOBJ_ACTUATOR, "actuator", "actuators" },
        { mjOBJ_SENSOR, "sensor", "sensors" },
        { mjOBJ_KEY, "keyframe", "keyframes" },
        { mjOBJ_FLEX, "flexible body", "flexible bodies" },
        { mjOBJ_SKIN, "skin", "skins" },
    };
    for (const Kind &kind : kinds) {
        int count = 0;
        for (mjsElement *e = mjs_firstElement(p_spec, kind.type); e; e = mjs_nextElement(p_spec, e)) {
            count++;
        }
        if (count > 0) {
            add_report("skipped_element", "", itos(count) + String(" ") + (count == 1 ? kind.singular : kind.plural)
                    + String(count == 1 ? " was" : " were") + String(" skipped: not supported yet."));
        }
    }

    // Default classes. MuJoCo doesn't offer a list of them, but every element
    // knows the class it takes its defaults from. Count the distinct classes
    // in use by the geoms and joints, other than the file's top-level one.
    const mjsDefault *top_level = mjs_getSpecDefault(p_spec);
    std::set<const mjsDefault *> classes;
    for (mjtObj type : { mjOBJ_GEOM, mjOBJ_JOINT }) {
        for (mjsElement *e = mjs_firstElement(p_spec, type); e; e = mjs_nextElement(p_spec, e)) {
            const mjsDefault *element_class = mjs_getDefault(e);
            if (element_class && element_class != top_level) {
                classes.insert(element_class);
            }
        }
    }
    if (!classes.empty()) {
        const int count = int(classes.size());
        add_report("flattened_default", "", itos(count) + String(count == 1 ? " default class was" : " default classes were")
                + String(" flattened: each element carries the values it inherited as its own."));
    }

    // Compiler settings that change masses after the fact.
    if (p_spec->compiler.settotalmass > 0 || p_spec->compiler.boundmass > 0 || p_spec->compiler.boundinertia > 0) {
        add_report("skipped_attribute", "", "The compiler's mass adjustments (settotalmass, boundmass, boundinertia) are not supported; masses will differ.");
    }
}

// ---- Verification ----

// How many timesteps both models are simulated for.
static const int VERIFY_STEPS = 200;

Array MujocoImporter::verify_file(MujocoWorld *p_world, const String &p_path)
{
    const String path = ProjectSettings::get_singleton()->globalize_path(p_path);
    char parse_error[1024] = "";
    mjSpec *spec = mj_parseXML(path.utf8().get_data(), nullptr, parse_error, sizeof(parse_error));
    return verify(p_world, spec, String::utf8(parse_error));
}

Array MujocoImporter::verify_string(MujocoWorld *p_world, const String &p_xml)
{
    char parse_error[1024] = "";
    mjSpec *spec = mj_parseXMLString(p_xml.utf8().get_data(), nullptr, parse_error, sizeof(parse_error));
    return verify(p_world, spec, String::utf8(parse_error));
}

// Takes ownership of p_spec, which is null if the MJCF couldn't be read.
Array MujocoImporter::verify(MujocoWorld *p_world, mjSpec *p_spec, const String &p_parse_error)
{
    // A problem with making the check, reported in the same form as a difference.
    const auto failure = [](const String &p_message) {
        Dictionary entry;
        entry["field"] = "error";
        entry["message"] = p_message;
        entry["count"] = 0;
        entry["max_difference"] = 0.0;
        Array result;
        result.push_back(entry);
        return result;
    };

    if (!p_spec) {
        return failure(String("Could not read the original MJCF: ") + p_parse_error);
    }
    mjModel *original = mj_compile(p_spec, nullptr);
    if (!original) {
        const String message = String("The original MJCF does not compile: ") + String::utf8(mjs_getError(p_spec));
        mj_deleteSpec(p_spec);
        return failure(message);
    }

    Array differences;
    if (!p_world || !p_world->get_model()) {
        differences = failure("The world to verify has no compiled model. Add it to the scene tree first.");
    } else {
        differences = mujoco_compare_models(original, p_world->get_model(), VERIFY_STEPS);
    }

    mj_deleteModel(original);
    mj_deleteSpec(p_spec);
    return differences;
}

String MujocoImporter::differences_to_text(const Array &p_differences)
{
    if (p_differences.is_empty()) {
        return "No differences.";
    }
    String text;
    for (int i = 0; i < p_differences.size(); i++) {
        const Dictionary entry = p_differences[i];
        text += String(entry["field"]) + String(": ") + String(entry["message"]) + String("\n");
    }
    return text;
}
