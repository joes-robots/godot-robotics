#include "mujoco_contact_exclude.h"
#include "mujoco_body.h"
#include "mujoco_world.h"

#include <godot_cpp/classes/engine.hpp>

using namespace godot;

String MujocoContactExclude::find_body(const NodePath &p_path, const char *p_property, String &r_problem) const
{
    const MujocoWorld *world = get_world();
    if (!world) {
        r_problem = "Not inside a MujocoWorld.";
        return String();
    }
    Node *node = p_path.is_empty() ? nullptr : get_node_or_null(p_path);
    if (node == world) {
        return "world";
    }
    MujocoBody *body = Object::cast_to<MujocoBody>(node);
    if (!body) {
        r_problem = vformat("\"%s\" must point to a MujocoBody (or the MujocoWorld).", p_property);
        return String();
    }
    if (body->get_world() != world) {
        r_problem = vformat("\"%s\" points to a body in a different MujocoWorld.", p_property);
        return String();
    }
    // Body names are their node paths relative to the world.
    return String(world->get_path_to(body));
}

bool MujocoContactExclude::find_bodies(String &r_body1, String &r_body2, String &r_problem) const
{
    r_body1 = find_body(body1, "body1", r_problem);
    if (r_body1.is_empty()) {
        return false;
    }
    r_body2 = find_body(body2, "body2", r_problem);
    if (r_body2.is_empty()) {
        return false;
    }
    if (r_body1 == r_body2) {
        r_problem = "body1 and body2 are the same body.";
        return false;
    }
    return true;
}

mjsBody *MujocoContactExclude::add_to_spec(mjSpec *p_spec, mjsBody *p_body, const Transform3D & /*p_transform*/, const String &p_name)
{
    String name1, name2, problem;
    if (!find_bodies(name1, name2, problem)) {
        // Left out, like an actuator without a joint.
        if (!Engine::get_singleton()->is_editor_hint()) {
            WARN_PRINT(vformat("MuJoCo contact exclusion '%s' is left out of the model: %s", p_name, problem));
        }
        return p_body;
    }
    mjsExclude *exclude = mjs_addExclude(p_spec);
    mjs_setName(exclude->element, p_name.utf8().get_data());
    mjs_setString(exclude->bodyname1, name1.utf8().get_data());
    mjs_setString(exclude->bodyname2, name2.utf8().get_data());
    return p_body;
}

PackedStringArray MujocoContactExclude::_get_configuration_warnings() const
{
    PackedStringArray warnings = MujocoElement::_get_configuration_warnings();
    String name1, name2, problem;
    if (get_world() && !find_bodies(name1, name2, problem)) {
        warnings.push_back(problem + " This exclusion is left out of the model.");
    }
    return warnings;
}

void MujocoContactExclude::set_body1(const NodePath &p_body)
{
    body1 = p_body;
    update_configuration_warnings();
    request_compile();
}

NodePath MujocoContactExclude::get_body1() const { return body1; }

void MujocoContactExclude::set_body2(const NodePath &p_body)
{
    body2 = p_body;
    update_configuration_warnings();
    request_compile();
}

NodePath MujocoContactExclude::get_body2() const { return body2; }

void MujocoContactExclude::_bind_methods()
{
    ClassDB::bind_method(D_METHOD("set_body1", "body"), &MujocoContactExclude::set_body1);
    ClassDB::bind_method(D_METHOD("get_body1"), &MujocoContactExclude::get_body1);
    ClassDB::bind_method(D_METHOD("set_body2", "body"), &MujocoContactExclude::set_body2);
    ClassDB::bind_method(D_METHOD("get_body2"), &MujocoContactExclude::get_body2);
    ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "body1", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "MujocoBody,MujocoWorld"), "set_body1", "get_body1");
    ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "body2", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "MujocoBody,MujocoWorld"), "set_body2", "get_body2");
}
