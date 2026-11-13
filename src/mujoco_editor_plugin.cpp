#include "mujoco_editor_plugin.h"

#include <godot_cpp/classes/mesh.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/classes/triangle_mesh.hpp>

#include "mujoco_geom.h"
#include "mujoco_joint.h"

using namespace godot;

bool MujocoGeomGizmoPlugin::_has_gizmo(Node3D *p_node) const
{
    return Object::cast_to<MujocoGeom>(p_node) != nullptr;
}

String MujocoGeomGizmoPlugin::_get_gizmo_name() const
{
    return "MujocoGeom";
}

// Called by the editor when a geom's gizmo needs rebuilding; the geom asks for
// that with update_gizmos() whenever its shape changes.
void MujocoGeomGizmoPlugin::_redraw(const Ref<EditorNode3DGizmo> &p_gizmo)
{
    p_gizmo->clear();

    const MujocoGeom *geom = Object::cast_to<MujocoGeom>(p_gizmo->get_node_3d());
    if (!geom) {
        return;
    }
    // Nothing is drawn here: the geom draws itself. This only adds what a click
    // is tested against, so a geom stays clickable with draw_shape off.
    const Ref<Mesh> mesh = geom->get_shape_mesh();
    if (mesh.is_valid()) {
        p_gizmo->add_collision_triangles(mesh->generate_triangle_mesh());
    }
}

// ---- Joint gizmo ----

namespace {

// Each pair of points appended is one line segment.
void add_line(PackedVector3Array &r_lines, const Vector3 &p_from, const Vector3 &p_to)
{
    r_lines.push_back(p_from);
    r_lines.push_back(p_to);
}

// Two unit vectors at right angles to p_axis and to each other.
void perpendiculars(const Vector3 &p_axis, Vector3 &r_u, Vector3 &r_v)
{
    // Any vector not parallel to the axis will do as a starting point.
    const Vector3 other = Math::abs(p_axis.y) < 0.9f ? Vector3(0, 1, 0) : Vector3(1, 0, 0);
    r_u = p_axis.cross(other).normalized();
    r_v = p_axis.cross(r_u);
}

// A ring of the given radius around p_axis, centred on p_center.
void add_ring(PackedVector3Array &r_lines, const Vector3 &p_center, const Vector3 &p_axis, float p_radius)
{
    Vector3 u, v;
    perpendiculars(p_axis, u, v);
    const int segments = 32;
    const float full_turn = 6.2831853f;
    for (int i = 0; i < segments; i++) {
        const float a0 = full_turn * i / segments;
        const float a1 = full_turn * (i + 1) / segments;
        add_line(r_lines,
                p_center + p_radius * (u * Math::cos(a0) + v * Math::sin(a0)),
                p_center + p_radius * (u * Math::cos(a1) + v * Math::sin(a1)));
    }
}

// An arrowhead with its point at p_tip, pointing along p_direction.
void add_arrowhead(PackedVector3Array &r_lines, const Vector3 &p_tip, const Vector3 &p_direction, float p_size)
{
    Vector3 u, v;
    perpendiculars(p_direction, u, v);
    const Vector3 base = p_tip - p_direction * p_size;
    const float half_width = 0.4f * p_size;
    add_line(r_lines, p_tip, base + u * half_width);
    add_line(r_lines, p_tip, base - u * half_width);
    add_line(r_lines, p_tip, base + v * half_width);
    add_line(r_lines, p_tip, base - v * half_width);
}

} // namespace

bool MujocoJointGizmoPlugin::_has_gizmo(Node3D *p_node) const
{
    return Object::cast_to<MujocoJoint>(p_node) != nullptr;
}

String MujocoJointGizmoPlugin::_get_gizmo_name() const
{
    return "MujocoJoint";
}

void MujocoJointGizmoPlugin::_redraw(const Ref<EditorNode3DGizmo> &p_gizmo)
{
    p_gizmo->clear();

    const MujocoJoint *joint = Object::cast_to<MujocoJoint>(p_gizmo->get_node_3d());
    if (!joint) {
        return;
    }

    if (!material_created) {
        // The last-but-one argument, on_top, draws the lines over everything
        // else, so a joint inside a geom is still visible.
        create_material("mujoco_joint", Color(0.3, 0.85, 1.0), false, true);
        material_created = true;
    }

    // Everything here is in the joint node's own frame, with the anchor at the
    // origin. The axis attribute is in that frame too.
    const float size = joint->get_gizmo_size();
    const int type = joint->get("type");
    Vector3 axis = joint->get("axis");
    axis = axis.is_zero_approx() ? Vector3(0, 1, 0) : axis.normalized();

    PackedVector3Array lines;
    switch (type) {
        case MujocoJoint::TYPE_HINGE:
            add_line(lines, -size * axis, size * axis);
            add_arrowhead(lines, size * axis, axis, 0.2f * size);
            add_ring(lines, Vector3(), axis, 0.35f * size);
            break;
        case MujocoJoint::TYPE_SLIDE:
            add_line(lines, -size * axis, size * axis);
            add_arrowhead(lines, size * axis, axis, 0.2f * size);
            add_arrowhead(lines, -size * axis, -axis, 0.2f * size);
            break;
        case MujocoJoint::TYPE_BALL:
            add_ring(lines, Vector3(), Vector3(1, 0, 0), 0.5f * size);
            add_ring(lines, Vector3(), Vector3(0, 1, 0), 0.5f * size);
            add_ring(lines, Vector3(), Vector3(0, 0, 1), 0.5f * size);
            break;
        case MujocoJoint::TYPE_FREE:
            add_line(lines, Vector3(-0.5f * size, 0, 0), Vector3(0.5f * size, 0, 0));
            add_line(lines, Vector3(0, -0.5f * size, 0), Vector3(0, 0.5f * size, 0));
            add_line(lines, Vector3(0, 0, -0.5f * size), Vector3(0, 0, 0.5f * size));
            break;
    }

    p_gizmo->add_lines(lines, get_material("mujoco_joint", p_gizmo));
    // The same lines are what a click is tested against.
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
