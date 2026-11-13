#include "mujoco_body.h"

#include "mujoco_convert.h"

using namespace godot;

void MujocoBody::_bind_methods()
{
}

mjsBody *MujocoBody::add_to_spec(mjSpec *p_spec, mjsBody *p_body, const Transform3D &p_transform) const
{
    // nullptr: no default class yet, so the body starts from MuJoCo's defaults.
    mjsBody *body = mjs_addBody(p_body, nullptr);
    mjs_setName(body->element, get_mujoco_name().utf8().get_data());
    mujoco_convert::to_mujoco(p_transform, body->pos, body->quat);

    // This node's children are added to the new body.
    return body;
}

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

void MujocoBody::_notification(int p_what)
{
    // Moved by the user: where it is now is its new rest pose. (MujocoElement
    // receives this notification too, and asks for the recompile.)
    if (p_what == NOTIFICATION_LOCAL_TRANSFORM_CHANGED && !applying_simulation_pose) {
        simulated = false;
    }
}
