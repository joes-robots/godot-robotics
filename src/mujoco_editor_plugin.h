#pragma once

// Editor-only classes. They are registered at MODULE_INITIALIZATION_LEVEL_EDITOR,
// which only happens inside the Godot editor, never in an exported game.

#include <godot_cpp/classes/editor_node3d_gizmo.hpp>
#include <godot_cpp/classes/editor_node3d_gizmo_plugin.hpp>
#include <godot_cpp/classes/editor_plugin.hpp>

// Makes a MujocoGeom selectable by clicking it in the 3D viewport.
//
// The editor decides what a click hits by asking each node's "gizmo" for its
// collision triangles. The mesh a geom draws itself with is an internal child
// node, and the editor only gives gizmos to nodes that belong to the scene, so
// clicking that mesh selects nothing. This plugin gives the geom itself a gizmo
// whose collision triangles are its shape.
class MujocoGeomGizmoPlugin : public godot::EditorNode3DGizmoPlugin {
    GDCLASS(MujocoGeomGizmoPlugin, godot::EditorNode3DGizmoPlugin);

public:
    bool _has_gizmo(godot::Node3D *p_node) const override;
    godot::String _get_gizmo_name() const override;
    void _redraw(const godot::Ref<godot::EditorNode3DGizmo> &p_gizmo) override;

protected:
    static void _bind_methods() {}
};

// Installs the editor additions above while the editor is running.
class MujocoEditorPlugin : public godot::EditorPlugin {
    GDCLASS(MujocoEditorPlugin, godot::EditorPlugin);

public:
    void _enter_tree() override;
    void _exit_tree() override;

protected:
    static void _bind_methods() {}

private:
    godot::Ref<MujocoGeomGizmoPlugin> geom_gizmo_plugin;
};
