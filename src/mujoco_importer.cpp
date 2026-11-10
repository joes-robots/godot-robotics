#include "mujoco_importer.h"
#include "mujoco_actuator.h"
#include "mujoco_body.h"
#include "mujoco_contact_exclude.h"
#include "mujoco_convert.h"
#include "mujoco_default.h"
#include "mujoco_geom.h"
#include "mujoco_joint.h"
#include "mujoco_sensor.h"
#include "mujoco_site.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <map>

#include <godot_cpp/classes/dir_access.hpp>
#include <godot_cpp/classes/project_settings.hpp>
#include <godot_cpp/classes/resource_saver.hpp>
#include <godot_cpp/classes/xml_parser.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/variant/vector3.hpp>

using namespace godot;

namespace {

// --- Values -------------------------------------------------------------------

bool near(double p_a, double p_b, double p_tolerance = 1e-6)
{
    // Godot vectors and colors are 32-bit, so allow their rounding.
    return std::fabs(p_a - p_b) <= p_tolerance * std::max(1.0, std::max(std::fabs(p_a), std::fabs(p_b)));
}

bool same_value(const Variant &p_a, const Variant &p_b)
{
    switch (p_a.get_type()) {
        case Variant::FLOAT:
        case Variant::INT:
        case Variant::BOOL:
            return near(static_cast<double>(p_a), static_cast<double>(p_b));
        case Variant::VECTOR2: {
            const Vector2 a = p_a, b = p_b;
            return near(a.x, b.x) && near(a.y, b.y);
        }
        case Variant::VECTOR3: {
            const Vector3 a = p_a, b = p_b;
            return near(a.x, b.x) && near(a.y, b.y) && near(a.z, b.z);
        }
        case Variant::COLOR: {
            const Color a = p_a, b = p_b;
            return near(a.r, b.r) && near(a.g, b.g) && near(a.b, b.b) && near(a.a, b.a);
        }
        default:
            return p_a == p_b;
    }
}

// mjtLimited -> bool, as the compiler resolves "auto".
bool resolve_limited(int p_limited, const double p_range[2], bool p_autolimits)
{
    if (p_limited == mjLIMITED_AUTO) {
        return p_autolimits && p_range[0] < p_range[1];
    }
    return p_limited == mjLIMITED_TRUE;
}

bool is_primitive_shape(int p_type)
{
    return p_type == mjGEOM_PLANE || p_type == mjGEOM_SPHERE || p_type == mjGEOM_CAPSULE ||
            p_type == mjGEOM_ELLIPSOID || p_type == mjGEOM_CYLINDER || p_type == mjGEOM_BOX;
}

const char *geom_type_name(int p_type)
{
    switch (p_type) {
        case mjGEOM_HFIELD:
            return "Height field";
        case mjGEOM_SDF:
            return "SDF";
        default:
            return "Unknown";
    }
}

// Rotation taking the node's local Y axis (a joint's axis) to p_axis.
Basis basis_with_y_along(const Vector3 &p_axis)
{
    const Vector3 y(0, 1, 0);
    const Vector3 axis = p_axis.normalized();
    const real_t d = y.dot(axis);
    if (d > 1.0 - 1e-9) {
        return Basis();
    }
    if (d < -1.0 + 1e-9) {
        return Basis(Vector3(1, 0, 0), Math::PI);
    }
    return Basis(y.cross(axis).normalized(), std::acos(d));
}

// Attribute values (see the element tables) of a joint class.
Dictionary joint_values(const mjsJoint *p_joint, bool p_degree, bool p_autolimits)
{
    Dictionary values;
    values["type"] = static_cast<int64_t>(p_joint->type);
    values["limited"] = resolve_limited(p_joint->limited, p_joint->range, p_autolimits);
    // Classes keep the file's units; the compiler converts per joint type.
    const double scale = p_degree && (p_joint->type == mjJNT_HINGE || p_joint->type == mjJNT_BALL) ? Math::PI / 180.0 : 1.0;
    values["range"] = Vector2(p_joint->range[0] * scale, p_joint->range[1] * scale);
    values["damping"] = p_joint->damping[0];
    values["stiffness"] = p_joint->stiffness[0];
    values["armature"] = p_joint->armature;
    values["frictionloss"] = p_joint->frictionloss;
    values["ref"] = p_joint->ref * scale;
    values["springref"] = p_joint->springref * scale;
    return values;
}

// The contact attributes of a geom, from its solver parameters.
void add_contact_values(Dictionary &r_values, int p_condim, int p_priority, double p_margin, double p_gap, double p_solmix,
        const mjtNum *p_solref, const mjtNum *p_solimp)
{
    r_values["condim"] = static_cast<int64_t>(p_condim);
    r_values["priority"] = static_cast<int64_t>(p_priority);
    r_values["margin"] = p_margin;
    r_values["gap"] = p_gap;
    r_values["solmix"] = p_solmix;
    r_values["solref_timeconst"] = p_solref[0];
    r_values["solref_dampratio"] = p_solref[1];
    r_values["solimp_dmin"] = p_solimp[0];
    r_values["solimp_dmax"] = p_solimp[1];
    r_values["solimp_width"] = p_solimp[2];
    r_values["solimp_midpoint"] = p_solimp[3];
    r_values["solimp_power"] = p_solimp[4];
}

Dictionary geom_values(const mjsGeom *p_geom)
{
    Dictionary values;
    if (is_primitive_shape(p_geom->type) || p_geom->type == mjGEOM_MESH) {
        values["shape"] = static_cast<int64_t>(p_geom->type);
    }
    values["size0"] = p_geom->size[0];
    values["size1"] = p_geom->size[1];
    values["size2"] = p_geom->size[2];
    values["density"] = p_geom->density;
    values["mass"] = std::isnan(p_geom->mass) ? 0.0 : p_geom->mass;
    values["friction"] = Vector3(p_geom->friction[0], p_geom->friction[1], p_geom->friction[2]);
    values["collision_layer"] = static_cast<int64_t>(p_geom->contype);
    values["collision_mask"] = static_cast<int64_t>(p_geom->conaffinity);
    values["color"] = Color(p_geom->rgba[0], p_geom->rgba[1], p_geom->rgba[2], p_geom->rgba[3]);
    add_contact_values(values, p_geom->condim, p_geom->priority, p_geom->margin, p_geom->gap, p_geom->solmix, p_geom->solref, p_geom->solimp);
    return values;
}

Dictionary site_values(const mjsSite *p_site)
{
    Dictionary values;
    if (is_primitive_shape(p_site->type) && p_site->type != mjGEOM_PLANE) {
        values["shape"] = static_cast<int64_t>(p_site->type);
    }
    values["size0"] = p_site->size[0];
    values["size1"] = p_site->size[1];
    values["size2"] = p_site->size[2];
    return values;
}

// Gear and ranges of an actuator, and kp/kv when its gains look like a position or
// velocity servo (a class doesn't know which kind of actuator will use it).
Dictionary actuator_class_values(const mjsActuator *p_actuator, bool p_autolimits)
{
    Dictionary values;
    values["gear"] = p_actuator->gear[0];
    values["ctrl_limited"] = resolve_limited(p_actuator->ctrllimited, p_actuator->ctrlrange, p_autolimits);
    values["ctrl_range"] = Vector2(p_actuator->ctrlrange[0], p_actuator->ctrlrange[1]);
    values["force_limited"] = resolve_limited(p_actuator->forcelimited, p_actuator->forcerange, p_autolimits);
    values["force_range"] = Vector2(p_actuator->forcerange[0], p_actuator->forcerange[1]);
    if (p_actuator->biastype == mjBIAS_AFFINE) {
        if (p_actuator->biasprm[1] != 0.0) {
            values["kp"] = p_actuator->gainprm[0];
            values["kv"] = -p_actuator->biasprm[2];
        } else if (p_actuator->biasprm[2] != 0.0) {
            values["kv"] = p_actuator->gainprm[0];
        }
    }
    return values;
}

// The node of p_kind's element p_id, or nullptr.
template <typename T>
T *node_of(const std::vector<uint64_t> &p_nodes, int p_id)
{
    if (p_id < 0 || p_id >= static_cast<int>(p_nodes.size()) || p_nodes[p_id] == 0) {
        return nullptr;
    }
    return Object::cast_to<T>(ObjectDB::get_instance(p_nodes[p_id]));
}

// A mesh geom's vertices in its body's frame.
std::vector<std::array<double, 3>> mesh_geom_points(const mjModel *p_m, int p_geom)
{
    std::vector<std::array<double, 3>> points;
    const int mesh = p_m->geom_dataid[p_geom];
    if (mesh < 0) {
        return points;
    }
    const float *vertices = p_m->mesh_vert + 3 * p_m->mesh_vertadr[mesh];
    for (int v = 0; v < p_m->mesh_vertnum[mesh]; v++) {
        const mjtNum local[3] = { vertices[3 * v], vertices[3 * v + 1], vertices[3 * v + 2] };
        std::array<double, 3> point;
        mju_rotVecQuat(point.data(), local, p_m->geom_quat + 4 * p_geom);
        for (int k = 0; k < 3; k++) {
            point[k] += p_m->geom_pos[3 * p_geom + k];
        }
        points.push_back(point);
    }
    return points;
}

// The largest distance from a vertex of p_a (up to 200 of them) to the nearest
// vertex of p_b. The order of the vertices doesn't matter.
double farthest_point(const std::vector<std::array<double, 3>> &p_a, const std::vector<std::array<double, 3>> &p_b)
{
    if (p_a.empty() || p_b.empty()) {
        return p_a.empty() && p_b.empty() ? 0.0 : INFINITY;
    }
    double worst = 0.0;
    const size_t stride = std::max<size_t>(1, p_a.size() / 200);
    for (size_t i = 0; i < p_a.size(); i += stride) {
        double nearest = INFINITY;
        for (const std::array<double, 3> &b : p_b) {
            const double dx = p_a[i][0] - b[0], dy = p_a[i][1] - b[1], dz = p_a[i][2] - b[2];
            nearest = std::min(nearest, dx * dx + dy * dy + dz * dz);
        }
        worst = std::max(worst, nearest);
    }
    return std::sqrt(worst);
}

void set_owner_recursive(Node *p_node, Node *p_owner)
{
    const int count = p_node->get_child_count();
    for (int i = 0; i < count; i++) {
        Node *child = p_node->get_child(i);
        child->set_owner(p_owner);
        set_owner_recursive(child, p_owner);
    }
}

} // namespace

// --- Import -------------------------------------------------------------------

void MujocoImporter::reset()
{
    spec.reset();
    model.reset();
    class_names.clear();
    main_created = false;
    for (std::vector<uint64_t> &list : nodes) {
        list.clear();
    }
    meshes.clear();
    notes.clear();
}

void MujocoImporter::note(const String &p_what)
{
    for (std::pair<String, int> &entry : notes) {
        if (entry.first == p_what) {
            entry.second++;
            return;
        }
    }
    notes.emplace_back(p_what, 1);
}

PackedStringArray MujocoImporter::get_report() const
{
    PackedStringArray lines;
    for (const std::pair<String, int> &entry : notes) {
        lines.push_back(entry.second > 1 ? vformat("%s (%d)", entry.first, entry.second) : entry.first);
    }
    return lines;
}

MujocoWorld *MujocoImporter::import_file(const String &p_path)
{
    reset();
    // MuJoCo reads files itself (and resolves includes and assets next to them).
    const String path = ProjectSettings::get_singleton()->globalize_path(p_path);
    char error[1024] = "";
    mjSpec *parsed = mj_parseXML(path.utf8().get_data(), nullptr, error, sizeof(error));
    if (!parsed) {
        note(vformat("Could not load %s: %s", p_path, error));
        ERR_FAIL_V_MSG(nullptr, vformat("MuJoCo import: could not load %s: %s", p_path, error));
    }
    return import_spec(parsed);
}

MujocoWorld *MujocoImporter::import_string(const String &p_xml)
{
    reset();
    char error[1024] = "";
    mjSpec *parsed = mj_parseXMLString(p_xml.utf8().get_data(), nullptr, error, sizeof(error));
    if (!parsed) {
        note(vformat("Could not parse the XML: %s", error));
        ERR_FAIL_V_MSG(nullptr, vformat("MuJoCo import: could not parse the XML: %s", error));
    }
    return import_spec(parsed);
}

MujocoWorld *MujocoImporter::import_spec(mjSpec *p_spec)
{
    spec.reset(p_spec);
    // The compiled model has every value resolved: frames relative to the parent
    // body, sizes, angles in radians, inherited classes.
    model.reset(mj_compile(spec.get(), nullptr));
    if (!model) {
        const String error = mjs_getError(spec.get());
        note("The model does not compile: " + error);
        ERR_FAIL_V_MSG(nullptr, "MuJoCo import: the model does not compile: " + error);
    }
    nodes[KIND_BODY].assign(model->nbody, 0);
    nodes[KIND_JOINT].assign(model->njnt, 0);
    nodes[KIND_GEOM].assign(model->ngeom, 0);
    nodes[KIND_SITE].assign(model->nsite, 0);
    nodes[KIND_ACTUATOR].assign(model->nu, 0);
    nodes[KIND_SENSOR].assign(model->nsensor, 0);
    meshes.assign(model->nmesh, Ref<ArrayMesh>());

    MujocoWorld *world = memnew(MujocoWorld);
    const String model_name = mjs_getString(spec->modelname);
    world->set_name(model_name.is_empty() ? String("MujocoWorld") : model_name.validate_node_name());
    world->set_option(model->opt);

    import_classes(world);
    import_body_children(mjs_findBody(spec.get(), "world"), world, world, "main");
    import_actuators(world);
    import_sensors(world);
    import_excludes(world);
    note_unsupported_model_parts();

    set_owner_recursive(world, world);
    return world;
}

String MujocoImporter::node_name(mjsElement *p_element, const String &p_fallback)
{
    const String name = String::utf8(mjs_getString(mjs_getName(p_element)));
    return name.is_empty() ? p_fallback : name.validate_node_name();
}

String MujocoImporter::class_of(mjsElement *p_element) const
{
    const mjsDefault *def = mjs_getDefault(p_element);
    if (!def) {
        return "main";
    }
    const std::string name = mjs_getString(mjs_getName(def->element));
    const auto it = class_names.find(name);
    return it != class_names.end() ? it->second : String("main");
}

MujocoDefault *MujocoImporter::ensure_main_class(MujocoWorld *p_world)
{
    if (MujocoDefault *main = p_world->get_main_class()) {
        return main;
    }
    MujocoDefault *main = memnew(MujocoDefault);
    main->set_name("main");
    p_world->add_child(main);
    // Classes come first in the tree, like <default> in MJCF.
    p_world->move_child(main, 0);
    main_created = true;
    return main;
}

void MujocoImporter::import_classes(MujocoWorld *p_world)
{
    // MuJoCo's API doesn't expose a class's parent, but the saved MJCF nests the
    // classes, so read the tree from there; the values come from the spec.
    std::vector<char> buffer(1 << 16);
    char error[512] = "";
    int result = mj_saveXMLString(spec.get(), buffer.data(), static_cast<int>(buffer.size()), error, sizeof(error));
    if (result > 0) {
        buffer.resize(static_cast<size_t>(result) + 1);
        result = mj_saveXMLString(spec.get(), buffer.data(), static_cast<int>(buffer.size()), error, sizeof(error));
    }
    if (result != 0) {
        note(vformat("Default classes were not imported (%s); every element has its full values instead", error));
        return;
    }

    const bool degree = spec->compiler.degree != 0;
    const bool autolimits = spec->compiler.autolimits != 0;
    auto apply = [&](MujocoDefault *p_class, mjsDefault *p_def) {
        const Dictionary sections[] = {
            joint_values(p_def->joint, degree, autolimits),
            geom_values(p_def->geom),
            site_values(p_def->site),
            actuator_class_values(p_def->actuator, autolimits),
        };
        const char *names[] = { "joint", "geom", "site", "actuator" };
        for (int i = 0; i < 4; i++) {
            MujocoElement *section = p_class->get_section_element(names[i]);
            Dictionary overrides;
            const Array keys = sections[i].keys();
            for (int64_t k = 0; k < keys.size(); k++) {
                if (!same_value(section->get_attribute(keys[k]), sections[i][keys[k]])) {
                    overrides[keys[k]] = sections[i][keys[k]];
                }
            }
            section->set_overrides(overrides);
        }
    };

    Ref<XMLParser> parser;
    parser.instantiate();
    PackedByteArray bytes;
    bytes.resize(static_cast<int64_t>(std::strlen(buffer.data())));
    std::memcpy(bytes.ptrw(), buffer.data(), static_cast<size_t>(bytes.size()));
    parser->open_buffer(bytes);

    // The root class: MuJoCo's built-in defaults plus the file's top-level
    // <default>. A "main" node only if that differs from our base defaults (it
    // usually does: MuJoCo's default geom is a sphere, ours a box).
    class_names["main"] = "main";
    MujocoDefault *main = memnew(MujocoDefault);
    main->set_name("main");
    p_world->add_child(main);
    apply(main, mjs_getSpecDefault(spec.get()));
    if (main->get_overrides().is_empty()) {
        p_world->remove_child(main);
        memdelete(main);
    } else {
        main_created = true;
    }

    // The node each open <default> maps to (the world for the root class).
    std::vector<Node *> stack;
    while (parser->read() == OK) {
        if (parser->get_node_type() == XMLParser::NODE_ELEMENT_END) {
            if (parser->get_node_name() == "default" && !stack.empty()) {
                stack.pop_back();
            }
            continue;
        }
        if (parser->get_node_type() != XMLParser::NODE_ELEMENT || parser->get_node_name() != "default") {
            continue;
        }
        const String mujoco_name = parser->get_named_attribute_value_safe("class");
        Node *node = nullptr;
        if (stack.empty() || mujoco_name.is_empty() || mujoco_name == "main") {
            // The root class, handled above; its children are top-level classes.
            node = p_world;
        } else {
            MujocoDefault *cls = memnew(MujocoDefault);
            const String name = mujoco_name.validate_node_name();
            cls->set_name(name);
            // Top-level classes sit directly under the world and inherit from main.
            stack.back()->add_child(cls);
            class_names[mujoco_name.utf8().get_data()] = name;
            if (mjsDefault *def = mjs_findDefault(spec.get(), mujoco_name.utf8().get_data())) {
                apply(cls, def);
            }
            node = cls;
        }
        if (parser->is_empty()) {
            continue;
        }
        stack.push_back(node);
    }
}

void MujocoImporter::add_element(MujocoElement *p_element, Node *p_parent, const String &p_name, mjsElement *p_spec_element,
        const String &p_class, const Dictionary &p_values, MujocoWorld *p_world)
{
    p_element->set_name(p_name);
    p_parent->add_child(p_element, true);
    const String cls = class_of(p_spec_element);
    if (cls != p_class) {
        if (cls == "main") {
            ensure_main_class(p_world);
        }
        p_element->set_default_class(cls);
    }
    // Override only what differs from the class.
    Dictionary overrides;
    const Array keys = p_values.keys();
    for (int64_t i = 0; i < keys.size(); i++) {
        if (!same_value(p_element->get_attribute(keys[i]), p_values[keys[i]])) {
            overrides[keys[i]] = p_values[keys[i]];
        }
    }
    p_element->set_overrides(overrides);
}

void MujocoImporter::import_body_children(mjsBody *p_body, Node *p_node, MujocoWorld *p_world, const String &p_class)
{
    const mjModel *m = model.get();

    for (mjsElement *el = mjs_firstChild(p_body, mjOBJ_JOINT, 0); el; el = mjs_nextChild(p_body, el, 0)) {
        const int id = mjs_getId(el);
        const int dof = m->jnt_dofadr[id];
        const int qpos = m->jnt_qposadr[id];
        const int type = m->jnt_type[id];
        MujocoJoint *joint = memnew(MujocoJoint);
        if (type != mjJNT_FREE) {
            // The joint's local Y axis is its MuJoCo axis.
            joint->set_transform(Transform3D(basis_with_y_along(mujoco_convert::to_godot(m->jnt_axis + 3 * id)), mujoco_convert::to_godot(m->jnt_pos + 3 * id)));
        }
        Dictionary values;
        values["type"] = static_cast<int64_t>(type);
        values["limited"] = m->jnt_limited[id] != 0;
        values["range"] = Vector2(m->jnt_range[2 * id], m->jnt_range[2 * id + 1]);
        values["stiffness"] = m->jnt_stiffness[id];
        values["damping"] = m->dof_damping[dof];
        values["armature"] = m->dof_armature[dof];
        values["frictionloss"] = m->dof_frictionloss[dof];
        if (type == mjJNT_HINGE || type == mjJNT_SLIDE) {
            values["ref"] = m->qpos0[qpos];
            values["springref"] = m->qpos_spring[qpos];
        }
        const mjtNum *solref = m->jnt_solref + mjNREF * id;
        const mjtNum *solimp = m->jnt_solimp + mjNIMP * id;
        if (m->jnt_margin[id] != 0.0 || solref[0] != 0.02 || solref[1] != 1.0 || solimp[0] != 0.9 || solimp[1] != 0.95 ||
                solimp[2] != 0.001 || solimp[3] != 0.5 || solimp[4] != 2.0) {
            note("Joint limit solver parameters (solreflimit, solimplimit, margin) are not supported yet; MuJoCo's defaults are used");
        }
        if (type == mjJNT_FREE && (m->dof_damping[dof] != 0.0 || m->dof_armature[dof] != 0.0 || m->dof_frictionloss[dof] != 0.0)) {
            note("Free joint damping, armature and friction loss are not imported");
        }
        add_element(joint, p_node, node_name(el, "Joint"), el, p_class, values, p_world);
        nodes[KIND_JOINT][id] = joint->get_instance_id();
    }

    for (mjsElement *el = mjs_firstChild(p_body, mjOBJ_GEOM, 0); el; el = mjs_nextChild(p_body, el, 0)) {
        const int id = mjs_getId(el);
        const int type = m->geom_type[id];
        if (!is_primitive_shape(type) && type != mjGEOM_MESH) {
            note(vformat("%s geoms are not supported yet and were left out", geom_type_name(type)));
            continue;
        }
        const mjsGeom *spec_geom = mjs_asGeom(el);
        MujocoGeom *geom = memnew(MujocoGeom);
        Transform3D frame = mujoco_convert::to_godot(m->geom_pos + 3 * id, m->geom_quat + 4 * id);
        Dictionary values;
        values["shape"] = static_cast<int64_t>(type);
        if (type == mjGEOM_MESH) {
            // The compiled geom frame includes the mesh's own offset (MuJoCo moves the
            // vertices to their center of mass); the ArrayMesh keeps the file's frame.
            const int mesh_id = m->geom_dataid[id];
            frame = frame * mujoco_convert::to_godot(m->mesh_pos + 3 * mesh_id, m->mesh_quat + 4 * mesh_id).affine_inverse();
            geom->set_mesh(mesh_resource(mesh_id));
        } else {
            values["size0"] = m->geom_size[3 * id];
            values["size1"] = m->geom_size[3 * id + 1];
            values["size2"] = m->geom_size[3 * id + 2];
        }
        geom->set_transform(frame);
        if (spec_geom->typeinertia == mjINERTIA_SHELL) {
            note("Shell inertia (shellinertia) is not supported yet; the geom's mass is spread over its volume");
        }
        // The compiled model keeps only the body's mass, so these come from the spec.
        values["density"] = spec_geom->density;
        values["mass"] = std::isnan(spec_geom->mass) ? 0.0 : spec_geom->mass;
        const mjtNum *friction = m->geom_friction + 3 * id;
        values["friction"] = Vector3(friction[0], friction[1], friction[2]);
        values["collision_layer"] = static_cast<int64_t>(m->geom_contype[id]);
        values["collision_mask"] = static_cast<int64_t>(m->geom_conaffinity[id]);
        const float *rgba = m->geom_rgba + 4 * id;
        values["color"] = Color(rgba[0], rgba[1], rgba[2], rgba[3]);
        if (m->geom_matid[id] >= 0) {
            note("Materials and textures are not imported; geoms keep their rgba color");
        }
        add_contact_values(values, m->geom_condim[id], m->geom_priority[id], m->geom_margin[id], m->geom_gap[id], m->geom_solmix[id],
                m->geom_solref + mjNREF * id, m->geom_solimp + mjNIMP * id);
        add_element(geom, p_node, node_name(el, "Geom"), el, p_class, values, p_world);
        nodes[KIND_GEOM][id] = geom->get_instance_id();
    }

    for (mjsElement *el = mjs_firstChild(p_body, mjOBJ_SITE, 0); el; el = mjs_nextChild(p_body, el, 0)) {
        const int id = mjs_getId(el);
        const int type = m->site_type[id];
        if (!is_primitive_shape(type) || type == mjGEOM_PLANE) {
            note("Sites with an unsupported shape were left out");
            continue;
        }
        MujocoSite *site = memnew(MujocoSite);
        site->set_transform(mujoco_convert::to_godot(m->site_pos + 3 * id, m->site_quat + 4 * id));
        Dictionary values;
        values["shape"] = static_cast<int64_t>(type);
        values["size0"] = m->site_size[3 * id];
        values["size1"] = m->site_size[3 * id + 1];
        values["size2"] = m->site_size[3 * id + 2];
        add_element(site, p_node, node_name(el, "Site"), el, p_class, values, p_world);
        nodes[KIND_SITE][id] = site->get_instance_id();
    }

    for (mjsElement *el = mjs_firstChild(p_body, mjOBJ_BODY, 0); el; el = mjs_nextChild(p_body, el, 0)) {
        mjsBody *spec_body = mjs_asBody(el);
        const int id = mjs_getId(el);
        MujocoBody *body = memnew(MujocoBody);
        body->set_name(node_name(el, "Body"));
        body->set_transform(mujoco_convert::to_godot(m->body_pos + 3 * id, m->body_quat + 4 * id));
        body->set_mocap(m->body_mocapid[id] >= 0);
        String child_class = p_class;
        const std::string spec_child_class = mjs_getString(spec_body->childclass);
        if (!spec_child_class.empty()) {
            const auto it = class_names.find(spec_child_class);
            child_class = it != class_names.end() ? it->second : String("main");
            if (child_class == "main") {
                ensure_main_class(p_world);
            }
            body->set_child_class(child_class);
        }
        if (spec_body->explicitinertial) {
            // The compiled inertia is diagonal, in a frame given by ipos and iquat.
            const Transform3D frame = mujoco_convert::to_godot(m->body_ipos + 3 * id, m->body_iquat + 4 * id);
            const mjtNum *inertia = m->body_inertia + 3 * id;
            body->set_explicit_inertia(true);
            body->set_mass(m->body_mass[id]);
            body->set_center_of_mass(frame.origin);
            body->set_inertia_orientation(frame.basis.get_rotation_quaternion());
            // MuJoCo's x, y, z moments are about Godot's x, -z, y axes of that frame.
            body->set_inertia(Vector3(inertia[0], inertia[2], inertia[1]));
        }
        if (m->body_gravcomp[id] != 0.0) {
            note("Gravity compensation is not supported yet");
        }
        p_node->add_child(body, true);
        nodes[KIND_BODY][id] = body->get_instance_id();
        import_body_children(spec_body, body, p_world, child_class);
    }
}

Ref<ArrayMesh> MujocoImporter::mesh_resource(int p_id)
{
    if (meshes[p_id].is_valid()) {
        return meshes[p_id];
    }
    const mjModel *m = model.get();
    mjsMesh *spec_mesh = mjs_asMesh(mjs_findElement(spec.get(), mjOBJ_MESH, mj_id2name(m, mjOBJ_MESH, p_id)));
    if (spec_mesh) {
        if (spec_mesh->inertia != mjMESH_INERTIA_LEGACY) {
            note("Mesh inertia modes other than legacy (convex, exact, shell) are not supported yet; legacy is used");
        }
        if (spec_mesh->maxhullvert != -1) {
            note("Mesh maxhullvert is not supported yet; the full convex hull is used");
        }
    }

    // MuJoCo keeps the scaled vertices in the mesh's inertial frame; mesh_pos and
    // mesh_quat take them back to the file's frame.
    const Transform3D to_file = mujoco_convert::to_godot(m->mesh_pos + 3 * p_id, m->mesh_quat + 4 * p_id);
    const float *vertices = m->mesh_vert + 3 * m->mesh_vertadr[p_id];
    const float *normals = m->mesh_normal + 3 * m->mesh_normaladr[p_id];
    const int *faces = m->mesh_face + 3 * m->mesh_faceadr[p_id];
    const int *face_normals = m->mesh_facenormal + 3 * m->mesh_faceadr[p_id];
    const int vertex_count = m->mesh_vertnum[p_id];
    const int normal_count = m->mesh_normalnum[p_id];

    // The first vertex_count vertices are MuJoCo's, in its order, so exporting the
    // mesh (which merges equal positions) gives MuJoCo back the same vertex list:
    // the convex hull, and so the contacts, depend on that order. Corners whose
    // normal differs from the first one at their position get a copy.
    PackedVector3Array godot_vertices;
    PackedVector3Array godot_normals;
    godot_vertices.resize(vertex_count);
    godot_normals.resize(vertex_count);
    std::vector<int> first_normal(static_cast<size_t>(vertex_count), -1);
    for (int v = 0; v < vertex_count; v++) {
        const mjtNum vertex[3] = { vertices[3 * v], vertices[3 * v + 1], vertices[3 * v + 2] };
        godot_vertices.set(v, to_file.xform(mujoco_convert::to_godot(vertex)));
        godot_normals.set(v, Vector3(0, 1, 0));
    }
    auto godot_normal = [&](int p_normal) {
        if (p_normal < 0 || p_normal >= normal_count) {
            return Vector3(0, 1, 0);
        }
        const mjtNum normal[3] = { normals[3 * p_normal], normals[3 * p_normal + 1], normals[3 * p_normal + 2] };
        return to_file.basis.xform(mujoco_convert::to_godot(normal)).normalized();
    };
    std::map<std::pair<int, int>, int> copies;
    PackedInt32Array indices;
    for (int f = 0; f < m->mesh_facenum[p_id]; f++) {
        // MuJoCo's faces are counterclockwise, Godot's clockwise.
        for (const int k : { 0, 2, 1 }) {
            const int v = faces[3 * f + k];
            const int n = face_normals[3 * f + k];
            ERR_CONTINUE(v < 0 || v >= vertex_count);
            int index = v;
            if (first_normal[v] < 0) {
                first_normal[v] = n;
                godot_normals.set(v, godot_normal(n));
            } else if (first_normal[v] != n) {
                const auto copy = copies.emplace(std::make_pair(v, n), static_cast<int>(godot_vertices.size()));
                if (copy.second) {
                    godot_vertices.push_back(godot_vertices[v]);
                    godot_normals.push_back(godot_normal(n));
                }
                index = copy.first->second;
            }
            indices.push_back(index);
        }
    }
    Array arrays;
    arrays.resize(Mesh::ARRAY_MAX);
    arrays[Mesh::ARRAY_VERTEX] = godot_vertices;
    arrays[Mesh::ARRAY_NORMAL] = godot_normals;
    arrays[Mesh::ARRAY_INDEX] = indices;
    Ref<ArrayMesh> mesh;
    mesh.instantiate();
    mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);
    const String name = String::utf8(mj_id2name(m, mjOBJ_MESH, p_id));
    mesh->set_name(name.is_empty() ? vformat("mesh%d", p_id) : name);
    meshes[p_id] = mesh;
    return mesh;
}

Error MujocoImporter::save_meshes(const String &p_directory)
{
    Error result = DirAccess::make_dir_recursive_absolute(p_directory);
    ERR_FAIL_COND_V_MSG(result != OK, result, "Could not create " + p_directory);
    PackedStringArray used;
    for (const Ref<ArrayMesh> &mesh : meshes) {
        if (mesh.is_null()) {
            continue;
        }
        String file = mesh->get_name().validate_filename();
        for (int i = 2; used.has(file); i++) {
            file = mesh->get_name().validate_filename() + "_" + itos(i);
        }
        used.push_back(file);
        const String path = p_directory.path_join(file + ".res");
        const Error error = ResourceSaver::get_singleton()->save(mesh, path);
        if (error != OK) {
            ERR_PRINT("Could not save " + path);
            result = error;
            continue;
        }
        mesh->take_over_path(path);
    }
    return result;
}

void MujocoImporter::import_actuators(MujocoWorld *p_world)
{
    const mjModel *m = model.get();
    for (mjsElement *el = mjs_firstElement(spec.get(), mjOBJ_ACTUATOR); el; el = mjs_nextElement(spec.get(), el)) {
        const int id = mjs_getId(el);
        const mjtNum *gain = m->actuator_gainprm + mjNGAIN * id;
        const mjtNum *bias = m->actuator_biasprm + mjNBIAS * id;
        MujocoJoint *joint = m->actuator_trntype[id] == mjTRN_JOINT ? node_of<MujocoJoint>(nodes[KIND_JOINT], m->actuator_trnid[2 * id]) : nullptr;
        if (!joint || (joint->get_type() != MujocoJoint::TYPE_HINGE && joint->get_type() != MujocoJoint::TYPE_SLIDE)) {
            note("Actuators other than on a hinge or slide joint (tendons, sites, ...) are not supported yet and were left out");
            continue;
        }
        if (m->actuator_dyntype[id] != mjDYN_NONE || m->actuator_gaintype[id] != mjGAIN_FIXED) {
            note("Actuators with activation dynamics or non-fixed gains (muscles, cylinders, ...) are not supported yet and were left out");
            continue;
        }
        // Which of our types the gains match (MuJoCo's <motor>, <position>, <velocity>).
        MujocoActuator::Type type;
        if (m->actuator_biastype[id] == mjBIAS_NONE && gain[0] == 1.0) {
            type = MujocoActuator::TYPE_MOTOR;
        } else if (m->actuator_biastype[id] == mjBIAS_AFFINE && bias[0] == 0.0 && bias[1] == -gain[0] && bias[1] != 0.0) {
            type = MujocoActuator::TYPE_POSITION;
        } else if (m->actuator_biastype[id] == mjBIAS_AFFINE && bias[0] == 0.0 && bias[1] == 0.0 && bias[2] == -gain[0]) {
            type = MujocoActuator::TYPE_VELOCITY;
        } else {
            note("General actuators whose gains are not a motor, position or velocity servo are not supported yet and were left out");
            continue;
        }
        Dictionary values;
        values["gear"] = m->actuator_gear[6 * id];
        if (type == MujocoActuator::TYPE_POSITION) {
            values["kp"] = gain[0];
            values["kv"] = -bias[2];
        } else if (type == MujocoActuator::TYPE_VELOCITY) {
            values["kv"] = gain[0];
        }
        values["ctrl_limited"] = m->actuator_ctrllimited[id] != 0;
        values["ctrl_range"] = Vector2(m->actuator_ctrlrange[2 * id], m->actuator_ctrlrange[2 * id + 1]);
        values["force_limited"] = m->actuator_forcelimited[id] != 0;
        values["force_range"] = Vector2(m->actuator_forcerange[2 * id], m->actuator_forcerange[2 * id + 1]);

        MujocoActuator *actuator = memnew(MujocoActuator);
        actuator->set_type(type);
        // Under its joint, which it drives by default (joint = "..").
        String joint_class = joint->get_default_class();
        if (joint_class.is_empty()) {
            const MujocoDefault *used = joint->get_used_class();
            joint_class = used ? String(used->get_name()) : String("main");
        }
        add_element(actuator, joint, node_name(el, "Actuator"), el, joint_class, values, p_world);
        nodes[KIND_ACTUATOR][id] = actuator->get_instance_id();
    }
}

void MujocoImporter::import_sensors(MujocoWorld *p_world)
{
    const mjModel *m = model.get();
    for (mjsElement *el = mjs_firstElement(spec.get(), mjOBJ_SENSOR); el; el = mjs_nextElement(spec.get(), el)) {
        const int id = mjs_getId(el);
        const int objtype = m->sensor_objtype[id];
        const int objid = m->sensor_objid[id];
        MujocoSensor::Type type;
        Node *target = nullptr;
        switch (m->sensor_type[id]) {
            case mjSENS_JOINTPOS:
                type = MujocoSensor::TYPE_JOINT_POSITION;
                target = node_of<Node>(nodes[KIND_JOINT], objid);
                break;
            case mjSENS_JOINTVEL:
                type = MujocoSensor::TYPE_JOINT_VELOCITY;
                target = node_of<Node>(nodes[KIND_JOINT], objid);
                break;
            case mjSENS_ACTUATORFRC:
                type = MujocoSensor::TYPE_ACTUATOR_FORCE;
                target = node_of<Node>(nodes[KIND_ACTUATOR], objid);
                break;
            case mjSENS_FRAMEPOS:
            case mjSENS_FRAMEQUAT:
            case mjSENS_FRAMELINVEL:
            case mjSENS_FRAMEANGVEL:
                type = m->sensor_type[id] == mjSENS_FRAMEPOS ? MujocoSensor::TYPE_FRAME_POSITION
                        : m->sensor_type[id] == mjSENS_FRAMEQUAT ? MujocoSensor::TYPE_FRAME_ORIENTATION
                        : m->sensor_type[id] == mjSENS_FRAMELINVEL ? MujocoSensor::TYPE_FRAME_LINEAR_VELOCITY
                                                                   : MujocoSensor::TYPE_FRAME_ANGULAR_VELOCITY;
                if (m->sensor_reftype[id] != mjOBJ_UNKNOWN) {
                    note("Frame sensors relative to a reference frame are not supported yet and were left out");
                    continue;
                }
                if (objtype == mjOBJ_SITE) {
                    target = node_of<Node>(nodes[KIND_SITE], objid);
                } else if (objtype == mjOBJ_XBODY) {
                    target = node_of<Node>(nodes[KIND_BODY], objid);
                } else {
                    note("Frame sensors on a body's inertial frame or on geoms and cameras are not supported yet and were left out");
                    continue;
                }
                break;
            case mjSENS_ACCELEROMETER:
                type = MujocoSensor::TYPE_ACCELEROMETER;
                target = node_of<Node>(nodes[KIND_SITE], objid);
                break;
            case mjSENS_GYRO:
                type = MujocoSensor::TYPE_GYRO;
                target = node_of<Node>(nodes[KIND_SITE], objid);
                break;
            case mjSENS_VELOCIMETER:
                type = MujocoSensor::TYPE_VELOCIMETER;
                target = node_of<Node>(nodes[KIND_SITE], objid);
                break;
            case mjSENS_TOUCH:
                type = MujocoSensor::TYPE_TOUCH;
                target = node_of<Node>(nodes[KIND_SITE], objid);
                break;
            default:
                note("Sensors of other types are not supported yet and were left out");
                continue;
        }
        if (!target) {
            note("Sensors whose target was left out were left out too");
            continue;
        }
        if (m->sensor_noise[id] != 0.0 || m->sensor_cutoff[id] != 0.0) {
            note("Sensor noise and cutoff are not supported yet");
        }
        MujocoSensor *sensor = memnew(MujocoSensor);
        sensor->set_type(type);
        sensor->set_name(node_name(el, "Sensor"));
        // Under its target, which it measures by default (target = "..").
        target->add_child(sensor, true);
        nodes[KIND_SENSOR][id] = sensor->get_instance_id();
    }
}

void MujocoImporter::import_excludes(MujocoWorld *p_world)
{
    for (mjsElement *el = mjs_firstElement(spec.get(), mjOBJ_EXCLUDE); el; el = mjs_nextElement(spec.get(), el)) {
        const mjsExclude *spec_exclude = mjs_asExclude(el);
        const int id1 = mj_name2id(model.get(), mjOBJ_BODY, mjs_getString(spec_exclude->bodyname1));
        const int id2 = mj_name2id(model.get(), mjOBJ_BODY, mjs_getString(spec_exclude->bodyname2));
        // Body 0 is the world.
        Node *body1 = id1 == 0 ? static_cast<Node *>(p_world) : node_of<Node>(nodes[KIND_BODY], id1);
        Node *body2 = id2 == 0 ? static_cast<Node *>(p_world) : node_of<Node>(nodes[KIND_BODY], id2);
        if (!body1 || !body2) {
            note("Contact exclusions involving left-out bodies were left out");
            continue;
        }
        MujocoContactExclude *exclude = memnew(MujocoContactExclude);
        exclude->set_name(node_name(el, "ContactExclude"));
        // Under body1, which it points to by default (body1 = "..").
        body1->add_child(exclude, true);
        if (body1 == p_world) {
            exclude->set_body1(NodePath(".."));
        }
        exclude->set_body2(exclude->get_path_to(body2));
    }
}

void MujocoImporter::note_unsupported_model_parts()
{
    const mjModel *m = model.get();
    const struct {
        mjtSize count;
        const char *what;
    } parts[] = {
        { m->ntendon, "Tendons are not supported yet and were left out" },
        { m->neq, "Equality constraints are not supported yet and were left out" },
        { m->nkey, "Keyframes are not supported yet and were left out" },
        { m->npair, "Explicit contact pairs are not supported yet and were left out" },
        { m->ncam, "Cameras were left out" },
        { m->nlight, "Lights were left out" },
        { m->nflex, "Flexible bodies are not supported and were left out" },
        { m->nplugin, "Plugins are not supported and were left out" },
    };
    for (const auto &part : parts) {
        if (part.count > 0) {
            note(vformat("%s (%d)", part.what, static_cast<int64_t>(part.count)));
        }
    }
}

// --- Verify -------------------------------------------------------------------

PackedStringArray MujocoImporter::verify(MujocoWorld *p_world, int p_steps)
{
    PackedStringArray diffs;
    ERR_FAIL_NULL_V(p_world, diffs);
    ERR_FAIL_COND_V_MSG(!model, diffs, "Nothing to verify: no model has been imported.");
    const mjModel *ours = p_world->get_model();
    ERR_FAIL_NULL_V_MSG(ours, diffs, "The world has no compiled model; add it to the scene tree first.");
    const mjModel *orig = model.get();
    const int max_diffs = 40;

    auto add = [&](const String &p_message) {
        if (diffs.size() < max_diffs) {
            diffs.push_back(p_message);
        }
    };
    auto name_of = [&](mjtObj p_type, int p_id) {
        const char *name = mj_id2name(orig, p_type, p_id);
        return name && name[0] ? String::utf8(name) : vformat("#%d", p_id);
    };
    auto our_id = [&](Kind p_kind, mjtObj p_type, int p_id) {
        Node *node = node_of<Node>(nodes[p_kind], p_id);
        return node ? mj_name2id(ours, p_type, String(p_world->get_path_to(node)).utf8().get_data()) : -1;
    };
    // Compares p_count numbers; positions get a little extra room for 32-bit transforms.
    auto compare = [&](const char *p_kind, const String &p_name, const char *p_field, const mjtNum *p_a, const mjtNum *p_b, int p_count, double p_tolerance) {
        for (int i = 0; i < p_count; i++) {
            if (!near(p_a[i], p_b[i], p_tolerance)) {
                String a, b;
                for (int j = 0; j < p_count; j++) {
                    a += String(j ? " " : "") + String::num(p_a[j], 6);
                    b += String(j ? " " : "") + String::num(p_b[j], 6);
                }
                add(vformat("%s '%s' %s: %s (file) vs %s (imported)", p_kind, p_name, p_field, a, b));
                return;
            }
        }
    };
    auto compare_quat = [&](const char *p_kind, const String &p_name, const mjtNum *p_a, const mjtNum *p_b) {
        // q and -q are the same rotation.
        const double dot = std::fabs(p_a[0] * p_b[0] + p_a[1] * p_b[1] + p_a[2] * p_b[2] + p_a[3] * p_b[3]);
        if (dot < 1.0 - 1e-6) {
            add(vformat("%s '%s' orientation differs (|q1.q2| = %f)", p_kind, p_name, dot));
        }
    };
    auto compare_int = [&](const char *p_kind, const String &p_name, const char *p_field, int p_a, int p_b) {
        if (p_a != p_b) {
            add(vformat("%s '%s' %s: %d (file) vs %d (imported)", p_kind, p_name, p_field, p_a, p_b));
        }
    };
    const double tolerance = 1e-5;

    if (std::memcmp(&orig->opt, &ours->opt, sizeof(mjOption)) != 0) {
        add("Physics options differ");
    }

    for (int i = 1; i < orig->nbody; i++) {
        const String name = name_of(mjOBJ_BODY, i);
        const int j = our_id(KIND_BODY, mjOBJ_BODY, i);
        if (j < 0) {
            add(vformat("Body '%s' was not imported", name));
            continue;
        }
        compare("Body", name, "position", orig->body_pos + 3 * i, ours->body_pos + 3 * j, 3, tolerance);
        compare_quat("Body", name, orig->body_quat + 4 * i, ours->body_quat + 4 * j);
        compare("Body", name, "mass", orig->body_mass + i, ours->body_mass + j, 1, tolerance);
        compare("Body", name, "inertia", orig->body_inertia + 3 * i, ours->body_inertia + 3 * j, 3, tolerance);
        compare("Body", name, "center of mass", orig->body_ipos + 3 * i, ours->body_ipos + 3 * j, 3, tolerance);
    }
    for (int i = 0; i < orig->njnt; i++) {
        const String name = name_of(mjOBJ_JOINT, i);
        const int j = our_id(KIND_JOINT, mjOBJ_JOINT, i);
        if (j < 0) {
            add(vformat("Joint '%s' was not imported", name));
            continue;
        }
        compare_int("Joint", name, "type", orig->jnt_type[i], ours->jnt_type[j]);
        compare_int("Joint", name, "limited", orig->jnt_limited[i], ours->jnt_limited[j]);
        if (orig->jnt_type[i] != mjJNT_FREE) {
            compare("Joint", name, "position", orig->jnt_pos + 3 * i, ours->jnt_pos + 3 * j, 3, tolerance);
            compare("Joint", name, "axis", orig->jnt_axis + 3 * i, ours->jnt_axis + 3 * j, 3, tolerance);
        }
        if (orig->jnt_limited[i]) {
            compare("Joint", name, "range", orig->jnt_range + 2 * i, ours->jnt_range + 2 * j, 2, tolerance);
        }
        compare("Joint", name, "stiffness", orig->jnt_stiffness + i, ours->jnt_stiffness + j, 1, tolerance);
        const int a = orig->jnt_dofadr[i], b = ours->jnt_dofadr[j];
        compare("Joint", name, "damping", orig->dof_damping + a, ours->dof_damping + b, 1, tolerance);
        compare("Joint", name, "armature", orig->dof_armature + a, ours->dof_armature + b, 1, tolerance);
        compare("Joint", name, "friction loss", orig->dof_frictionloss + a, ours->dof_frictionloss + b, 1, tolerance);
        const int qa = orig->jnt_qposadr[i], qb = ours->jnt_qposadr[j];
        if (orig->jnt_type[i] == mjJNT_HINGE || orig->jnt_type[i] == mjJNT_SLIDE) {
            compare("Joint", name, "ref", orig->qpos0 + qa, ours->qpos0 + qb, 1, tolerance);
            compare("Joint", name, "springref", orig->qpos_spring + qa, ours->qpos_spring + qb, 1, tolerance);
        }
    }
    for (int i = 0; i < orig->ngeom; i++) {
        const String name = name_of(mjOBJ_GEOM, i);
        const int j = our_id(KIND_GEOM, mjOBJ_GEOM, i);
        if (j < 0) {
            add(vformat("Geom '%s' was not imported", name));
            continue;
        }
        compare_int("Geom", name, "type", orig->geom_type[i], ours->geom_type[j]);
        if (orig->geom_type[i] == mjGEOM_MESH && ours->geom_type[j] == mjGEOM_MESH) {
            // A mesh geom's frame is the mesh's principal axes, which a symmetric mesh
            // doesn't pin down, so compare where the vertices are instead.
            const std::vector<std::array<double, 3>> a = mesh_geom_points(orig, i), b = mesh_geom_points(ours, j);
            const double distance = std::max(farthest_point(a, b), farthest_point(b, a));
            if (distance > tolerance * std::max(1.0, orig->geom_rbound[i])) {
                add(vformat("Geom '%s' mesh vertices are up to %f apart", name, distance));
            }
        } else {
            compare("Geom", name, "size", orig->geom_size + 3 * i, ours->geom_size + 3 * j, 3, tolerance);
            compare("Geom", name, "position", orig->geom_pos + 3 * i, ours->geom_pos + 3 * j, 3, tolerance);
            compare_quat("Geom", name, orig->geom_quat + 4 * i, ours->geom_quat + 4 * j);
        }
        compare("Geom", name, "friction", orig->geom_friction + 3 * i, ours->geom_friction + 3 * j, 3, tolerance);
        compare_int("Geom", name, "contype", orig->geom_contype[i], ours->geom_contype[j]);
        compare_int("Geom", name, "conaffinity", orig->geom_conaffinity[i], ours->geom_conaffinity[j]);
        compare_int("Geom", name, "condim", orig->geom_condim[i], ours->geom_condim[j]);
        compare_int("Geom", name, "priority", orig->geom_priority[i], ours->geom_priority[j]);
        compare("Geom", name, "margin", orig->geom_margin + i, ours->geom_margin + j, 1, tolerance);
        compare("Geom", name, "gap", orig->geom_gap + i, ours->geom_gap + j, 1, tolerance);
        compare("Geom", name, "solmix", orig->geom_solmix + i, ours->geom_solmix + j, 1, tolerance);
        compare("Geom", name, "solref", orig->geom_solref + mjNREF * i, ours->geom_solref + mjNREF * j, mjNREF, tolerance);
        compare("Geom", name, "solimp", orig->geom_solimp + mjNIMP * i, ours->geom_solimp + mjNIMP * j, mjNIMP, tolerance);
    }
    for (int i = 0; i < orig->nsite; i++) {
        const String name = name_of(mjOBJ_SITE, i);
        const int j = our_id(KIND_SITE, mjOBJ_SITE, i);
        if (j < 0) {
            add(vformat("Site '%s' was not imported", name));
            continue;
        }
        compare_int("Site", name, "type", orig->site_type[i], ours->site_type[j]);
        compare("Site", name, "size", orig->site_size + 3 * i, ours->site_size + 3 * j, 3, tolerance);
        compare("Site", name, "position", orig->site_pos + 3 * i, ours->site_pos + 3 * j, 3, tolerance);
        compare_quat("Site", name, orig->site_quat + 4 * i, ours->site_quat + 4 * j);
    }
    for (int i = 0; i < orig->nu; i++) {
        const String name = name_of(mjOBJ_ACTUATOR, i);
        const int j = our_id(KIND_ACTUATOR, mjOBJ_ACTUATOR, i);
        if (j < 0) {
            add(vformat("Actuator '%s' was not imported", name));
            continue;
        }
        const int target = our_id(KIND_JOINT, mjOBJ_JOINT, orig->actuator_trnid[2 * i]);
        compare_int("Actuator", name, "joint", target, ours->actuator_trnid[2 * j]);
        compare("Actuator", name, "gear", orig->actuator_gear + 6 * i, ours->actuator_gear + 6 * j, 6, tolerance);
        compare("Actuator", name, "gain", orig->actuator_gainprm + mjNGAIN * i, ours->actuator_gainprm + mjNGAIN * j, 3, tolerance);
        compare("Actuator", name, "bias", orig->actuator_biasprm + mjNBIAS * i, ours->actuator_biasprm + mjNBIAS * j, 3, tolerance);
        compare_int("Actuator", name, "ctrllimited", orig->actuator_ctrllimited[i], ours->actuator_ctrllimited[j]);
        compare_int("Actuator", name, "forcelimited", orig->actuator_forcelimited[i], ours->actuator_forcelimited[j]);
        if (orig->actuator_ctrllimited[i]) {
            compare("Actuator", name, "ctrlrange", orig->actuator_ctrlrange + 2 * i, ours->actuator_ctrlrange + 2 * j, 2, tolerance);
        }
        if (orig->actuator_forcelimited[i]) {
            compare("Actuator", name, "forcerange", orig->actuator_forcerange + 2 * i, ours->actuator_forcerange + 2 * j, 2, tolerance);
        }
    }
    for (int i = 0; i < orig->nsensor; i++) {
        const String name = name_of(mjOBJ_SENSOR, i);
        const int j = our_id(KIND_SENSOR, mjOBJ_SENSOR, i);
        if (j < 0) {
            add(vformat("Sensor '%s' was not imported", name));
            continue;
        }
        compare_int("Sensor", name, "type", orig->sensor_type[i], ours->sensor_type[j]);
    }

    // Contact exclusions, as unordered body pairs.
    {
        std::vector<std::pair<int, int>> our_pairs;
        for (int k = 0; k < ours->nexclude; k++) {
            const int a = ours->exclude_signature[k] >> 16, b = ours->exclude_signature[k] & 0xFFFF;
            our_pairs.emplace_back(std::min(a, b), std::max(a, b));
        }
        for (int k = 0; k < orig->nexclude; k++) {
            const int a = orig->exclude_signature[k] >> 16, b = orig->exclude_signature[k] & 0xFFFF;
            const int oa = a == 0 ? 0 : our_id(KIND_BODY, mjOBJ_BODY, a);
            const int ob = b == 0 ? 0 : our_id(KIND_BODY, mjOBJ_BODY, b);
            if (std::find(our_pairs.begin(), our_pairs.end(), std::make_pair(std::min(oa, ob), std::max(oa, ob))) == our_pairs.end()) {
                add(vformat("Contact exclusion between '%s' and '%s' is missing", name_of(mjOBJ_BODY, a), name_of(mjOBJ_BODY, b)));
            }
        }
        if (ours->nexclude != orig->nexclude) {
            add(vformat("%d contact exclusions in the file, %d imported", static_cast<int>(orig->nexclude), static_cast<int>(ours->nexclude)));
        }
    }

    // Both simulations from their initial state, without controls.
    if (p_steps > 0) {
        std::unique_ptr<mjData, MjDataDeleter> da(mj_makeData(orig));
        std::unique_ptr<mjData, MjDataDeleter> db(mj_makeData(ours));
        ERR_FAIL_COND_V(!da || !db, diffs);
        for (int step = 0; step < p_steps; step++) {
            mj_step(orig, da.get());
            mj_step(ours, db.get());
        }
        double worst = 0.0;
        String worst_name;
        for (int i = 0; i < orig->njnt; i++) {
            const int j = our_id(KIND_JOINT, mjOBJ_JOINT, i);
            if (j < 0) {
                continue;
            }
            const int type = orig->jnt_type[i];
            const int count = type == mjJNT_FREE ? 7 : type == mjJNT_BALL ? 4 : 1;
            const mjtNum *a = da->qpos + orig->jnt_qposadr[i];
            const mjtNum *b = db->qpos + ours->jnt_qposadr[j];
            // Quaternions: compare q and -q alike.
            const int quat_start = type == mjJNT_FREE ? 3 : type == mjJNT_BALL ? 0 : count;
            double sign = 1.0;
            if (quat_start < count && a[quat_start] * b[quat_start] + a[quat_start + 1] * b[quat_start + 1] + a[quat_start + 2] * b[quat_start + 2] + a[quat_start + 3] * b[quat_start + 3] < 0.0) {
                sign = -1.0;
            }
            for (int k = 0; k < count; k++) {
                const double diff = std::fabs(a[k] - (k >= quat_start ? sign : 1.0) * b[k]);
                if (diff > worst) {
                    worst = diff;
                    worst_name = name_of(mjOBJ_JOINT, i);
                }
            }
        }
        if (worst > 1e-3) {
            add(vformat("After %d steps, joint '%s' differs by %f (the largest difference)", p_steps, worst_name, worst));
        }
    }
    return diffs;
}

void MujocoImporter::_bind_methods()
{
    ClassDB::bind_method(D_METHOD("import_file", "path"), &MujocoImporter::import_file);
    ClassDB::bind_method(D_METHOD("import_string", "xml"), &MujocoImporter::import_string);
    ClassDB::bind_method(D_METHOD("get_report"), &MujocoImporter::get_report);
    ClassDB::bind_method(D_METHOD("verify", "world", "steps"), &MujocoImporter::verify, DEFVAL(1000));
    ClassDB::bind_method(D_METHOD("save_meshes", "directory"), &MujocoImporter::save_meshes);
}
