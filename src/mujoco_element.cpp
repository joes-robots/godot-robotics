#include "mujoco_element.h"
#include "mujoco_world.h"

#include <godot_cpp/classes/engine.hpp>

using namespace godot;

mjsBody *MujocoElement::add_to_spec(mjSpec * /*p_spec*/, mjsBody *p_body, const Transform3D & /*p_transform*/, const String & /*p_name*/)
{
    return p_body;
}

Transform3D MujocoElement::get_model_transform()
{
    return get_transform();
}

MujocoWorld *MujocoElement::get_world() const
{
    for (Node *node = get_parent(); node; node = node->get_parent()) {
        if (MujocoWorld *world = Object::cast_to<MujocoWorld>(node)) {
            return world;
        }
    }
    return nullptr;
}

void MujocoElement::request_compile() const
{
    if (MujocoWorld *world = get_world()) {
        world->request_compile();
    }
}

int MujocoElement::get_model_id(mjtObj p_type)
{
    MujocoWorld *world = get_world();
    if (!world) {
        return -1;
    }
    const mjModel *model = world->get_model();
    if (!model) {
        return -1;
    }
    // Element names are node paths relative to the world (see MujocoWorld::add_subtree).
    if (cached_generation != world->get_model_generation()) {
        cached_id = mj_name2id(model, p_type, String(world->get_path_to(this)).utf8().get_data());
        cached_generation = world->get_model_generation();
    }
    return cached_id;
}

PackedStringArray MujocoElement::_get_configuration_warnings() const
{
    PackedStringArray warnings = Node3D::_get_configuration_warnings();

    MujocoWorld *world = get_world();
    if (!world) {
        warnings.push_back("Must be a descendant of a MujocoWorld node.");
    } else {
        for (Node *node = get_parent(); node != world; node = node->get_parent()) {
            if (!Object::cast_to<Node3D>(node)) {
                warnings.push_back(vformat("All nodes between this node and its MujocoWorld must be Node3D, but '%s' is not. This node is ignored.", node->get_name()));
                break;
            }
        }
    }

    const Vector3 scale = get_scale();
    if (!scale.is_equal_approx(Vector3(1, 1, 1))) {
        warnings.push_back("MuJoCo does not support scale; it is ignored. Use this node's size properties instead.");
    }
    return warnings;
}

void MujocoElement::_notification(int p_what)
{
    switch (p_what) {
        case NOTIFICATION_ENTER_TREE:
            // In the editor, moving this node or any ancestor changes the model.
            // At runtime the simulation drives transforms, so don't listen.
            set_notify_transform(Engine::get_singleton()->is_editor_hint());
            update_configuration_warnings();
            request_compile();
            break;
        case NOTIFICATION_EXIT_TREE:
            // The parent chain is still intact here, so the world can be found.
            request_compile();
            break;
        case NOTIFICATION_TRANSFORM_CHANGED:
            update_configuration_warnings();
            request_compile();
            break;
        case NOTIFICATION_PATH_RENAMED:
        case NOTIFICATION_CHILD_ORDER_CHANGED:
            request_compile();
            break;
    }
}
