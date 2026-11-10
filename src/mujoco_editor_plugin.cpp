#include "mujoco_editor_plugin.h"
#include "mujoco_geom.h"
#include "mujoco_joint.h"

#include <cmath>

#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/classes/triangle_mesh.hpp>

using namespace godot;

bool MujocoGeomGizmoPlugin::_has_gizmo(Node3D *p_node) const
{
    return Object::cast_to<MujocoGeom>(p_node) != nullptr;
}

String MujocoGeomGizmoPlugin::_get_gizmo_name() const
{
    return "MujocoGeom";
}

void MujocoGeomGizmoPlugin::_redraw(const Ref<EditorNode3DGizmo> &p_gizmo)
{
    p_gizmo->clear();

    const MujocoGeom *geom = Object::cast_to<MujocoGeom>(p_gizmo->get_node_3d());
    if (!geom) {
        return;
    }
    const Ref<Mesh> mesh = geom->get_shape_mesh();
    if (mesh.is_valid()) {
        // Pickable even when show_shape is off.
        p_gizmo->add_collision_triangles(mesh->generate_triangle_mesh());
    }
}

bool MujocoJointGizmoPlugin::_has_gizmo(Node3D *p_node) const
{
    return Object::cast_to<MujocoJoint>(p_node) != nullptr;
}

String MujocoJointGizmoPlugin::_get_gizmo_name() const
{
    return "MujocoJoint";
}

// Appends a circle of radius p_radius in the plane spanned by p_a and p_b, as line segments.
static void add_circle(PackedVector3Array &r_lines, const Vector3 &p_a, const Vector3 &p_b, real_t p_radius)
{
    const int segments = 24;
    for (int i = 0; i < segments; i++) {
        const real_t t0 = Math::TAU * i / segments;
        const real_t t1 = Math::TAU * (i + 1) / segments;
        r_lines.push_back((p_a * std::cos(t0) + p_b * std::sin(t0)) * p_radius);
        r_lines.push_back((p_a * std::cos(t1) + p_b * std::sin(t1)) * p_radius);
    }
}

static void add_segment(PackedVector3Array &r_lines, const Vector3 &p_from, const Vector3 &p_to)
{
    r_lines.push_back(p_from);
    r_lines.push_back(p_to);
}

void MujocoJointGizmoPlugin::_redraw(const Ref<EditorNode3DGizmo> &p_gizmo)
{
    p_gizmo->clear();

    const MujocoJoint *joint = Object::cast_to<MujocoJoint>(p_gizmo->get_node_3d());
    if (!joint) {
        return;
    }
    if (!material_created) {
        // Same blue as Godot's own joint gizmos.
        create_material("joint", Color(0.5, 0.8, 1.0));
        material_created = true;
    }

    const Vector3 x(1, 0, 0);
    const Vector3 y(0, 1, 0); // the joint axis
    const Vector3 z(0, 0, 1);
    PackedVector3Array lines;

    switch (joint->get_type()) {
        case MujocoJoint::TYPE_HINGE: {
            // Axis, a circle around it, and an arrowhead for positive rotation
            // (right-hand rule: at +X the circle moves toward -Z).
            add_segment(lines, y * -0.25, y * 0.25);
            add_circle(lines, x, z, 0.12);
            const Vector3 tip = x * 0.12;
            add_segment(lines, tip, tip + Vector3(0.03, 0, 0.04));
            add_segment(lines, tip, tip + Vector3(-0.03, 0, 0.04));
        } break;
        case MujocoJoint::TYPE_SLIDE: {
            // Axis with an arrowhead at the positive end.
            const Vector3 tip = y * 0.3;
            add_segment(lines, y * -0.3, tip);
            add_segment(lines, tip, Vector3(0.04, 0.25, 0));
            add_segment(lines, tip, Vector3(-0.04, 0.25, 0));
            add_segment(lines, tip, Vector3(0, 0.25, 0.04));
            add_segment(lines, tip, Vector3(0, 0.25, -0.04));
        } break;
        case MujocoJoint::TYPE_BALL:
            add_circle(lines, x, y, 0.1);
            add_circle(lines, y, z, 0.1);
            add_circle(lines, x, z, 0.1);
            break;
        case MujocoJoint::TYPE_FREE:
            add_segment(lines, x * -0.1, x * 0.1);
            add_segment(lines, y * -0.1, y * 0.1);
            add_segment(lines, z * -0.1, z * 0.1);
            break;
    }

    p_gizmo->add_lines(lines, get_material("joint", p_gizmo));
    p_gizmo->add_collision_segments(lines);
}

void MujocoEditorPlugin::_enter_tree()
{
    geom_gizmo_plugin.instantiate();
    add_node_3d_gizmo_plugin(geom_gizmo_plugin);
    joint_gizmo_plugin.instantiate();
    add_node_3d_gizmo_plugin(joint_gizmo_plugin);
}

void MujocoEditorPlugin::_exit_tree()
{
    remove_node_3d_gizmo_plugin(joint_gizmo_plugin);
    joint_gizmo_plugin.unref();
    remove_node_3d_gizmo_plugin(geom_gizmo_plugin);
    geom_gizmo_plugin.unref();
}
