#include "mujoco_body.h"
#include "mujoco_convert.h"

#include <godot_cpp/classes/engine.hpp>

using namespace godot;

mjsBody *MujocoBody::add_to_spec(mjSpec * /*p_spec*/, mjsBody *p_body, const Transform3D &p_transform, const String &p_name)
{
    mjsBody *body = mjs_addBody(p_body, nullptr);
    mjs_setName(body->element, p_name.utf8().get_data());
    mujoco_convert::to_mujoco(p_transform, body->pos, body->quat);
    return body;
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

void MujocoBody::_notification(int p_what)
{
    if (p_what == NOTIFICATION_EXIT_TREE) {
        // A new parent means a new reference frame; capture again next time.
        has_rest_transform = false;
    }
}
