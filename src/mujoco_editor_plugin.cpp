#include "mujoco_editor_plugin.h"
#include "mujoco_geom.h"

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

void MujocoEditorPlugin::_enter_tree()
{
    geom_gizmo_plugin.instantiate();
    add_node_3d_gizmo_plugin(geom_gizmo_plugin);
}

void MujocoEditorPlugin::_exit_tree()
{
    remove_node_3d_gizmo_plugin(geom_gizmo_plugin);
    geom_gizmo_plugin.unref();
}
