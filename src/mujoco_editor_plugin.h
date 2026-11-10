#pragma once

// Editor-only classes. They are registered at MODULE_INITIALIZATION_LEVEL_EDITOR,
// which only runs in editor builds of Godot, never in exported games.

#include <godot_cpp/classes/editor_node3d_gizmo.hpp>
#include <godot_cpp/classes/editor_node3d_gizmo_plugin.hpp>
#include <godot_cpp/classes/editor_plugin.hpp>

// Gives every MujocoGeom a gizmo whose collision triangles are the geom's drawn
// shape, so the geom can be selected by clicking it in the 3D viewport. (The
// editor only creates gizmos for nodes saved in the scene, so the geom's
// internal MeshInstance3D can't be clicked on its own.)
class MujocoGeomGizmoPlugin : public godot::EditorNode3DGizmoPlugin {
    GDCLASS(MujocoGeomGizmoPlugin, godot::EditorNode3DGizmoPlugin);

public:
    bool _has_gizmo(godot::Node3D *p_node) const override;
    godot::String _get_gizmo_name() const override;
    void _redraw(const godot::Ref<godot::EditorNode3DGizmo> &p_gizmo) override;

protected:
    static void _bind_methods() {}
};

// Draws each MujocoJoint as lines showing its type and axis (the node's local Y),
// with an arrowhead for the positive direction. The lines are also clickable.
class MujocoJointGizmoPlugin : public godot::EditorNode3DGizmoPlugin {
    GDCLASS(MujocoJointGizmoPlugin, godot::EditorNode3DGizmoPlugin);

public:
    bool _has_gizmo(godot::Node3D *p_node) const override;
    godot::String _get_gizmo_name() const override;
    void _redraw(const godot::Ref<godot::EditorNode3DGizmo> &p_gizmo) override;

protected:
    static void _bind_methods() {}

private:
    bool material_created = false;
};

// Draws each MujocoSite's shape as a translucent orange mesh (sites are invisible
// in the running game), which can also be clicked to select the site.
class MujocoSiteGizmoPlugin : public godot::EditorNode3DGizmoPlugin {
    GDCLASS(MujocoSiteGizmoPlugin, godot::EditorNode3DGizmoPlugin);

public:
    bool _has_gizmo(godot::Node3D *p_node) const override;
    godot::String _get_gizmo_name() const override;
    void _redraw(const godot::Ref<godot::EditorNode3DGizmo> &p_gizmo) override;

protected:
    static void _bind_methods() {}

private:
    bool material_created = false;
};

// Registers the editor additions above while the editor is running.
class MujocoEditorPlugin : public godot::EditorPlugin {
    GDCLASS(MujocoEditorPlugin, godot::EditorPlugin);

public:
    void _enter_tree() override;
    void _exit_tree() override;

protected:
    static void _bind_methods() {}

private:
    godot::Ref<MujocoGeomGizmoPlugin> geom_gizmo_plugin;
    godot::Ref<MujocoJointGizmoPlugin> joint_gizmo_plugin;
    godot::Ref<MujocoSiteGizmoPlugin> site_gizmo_plugin;
};
