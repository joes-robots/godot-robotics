#pragma once

#include "mujoco_element.h"

#include <godot_cpp/variant/node_path.hpp>

// Turns off contacts between two bodies (MJCF's <contact><exclude>), e.g. two
// links that touch near the joint between them. (MuJoCo already ignores contacts
// between a body and its parent.)
//
// body1 and body2 point to MujocoBody nodes, or to the MujocoWorld for the world
// body (its static geoms). By default body1 is the parent node, so the exclusion
// can sit under one of the bodies. The node's transform is not used.
class MujocoContactExclude : public MujocoElement {
    GDCLASS(MujocoContactExclude, MujocoElement);

public:
    mjsBody *add_to_spec(mjSpec *p_spec, mjsBody *p_body, const godot::Transform3D &p_transform, const godot::String &p_name) override;
    godot::PackedStringArray _get_configuration_warnings() const override;

    void set_body1(const godot::NodePath &p_body);
    godot::NodePath get_body1() const;
    void set_body2(const godot::NodePath &p_body);
    godot::NodePath get_body2() const;

protected:
    static void _bind_methods();

private:
    // The MuJoCo body name p_path points to ("world" for the world), or an empty
    // string with the reason in r_problem.
    godot::String find_body(const godot::NodePath &p_path, const char *p_property, godot::String &r_problem) const;
    // Both names, or false with the reason in r_problem.
    bool find_bodies(godot::String &r_body1, godot::String &r_body2, godot::String &r_problem) const;

    godot::NodePath body1 = godot::NodePath("..");
    godot::NodePath body2;
};
