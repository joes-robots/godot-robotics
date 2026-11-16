#pragma once

// Editor-only classes. They are registered at MODULE_INITIALIZATION_LEVEL_EDITOR,
// which only happens inside the Godot editor, never in an exported game.

#include <godot_cpp/classes/editor_node3d_gizmo.hpp>
#include <godot_cpp/classes/editor_node3d_gizmo_plugin.hpp>
#include <godot_cpp/classes/editor_plugin.hpp>

namespace godot {
class AcceptDialog;
class EditorFileDialog;
class RichTextLabel;
}

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

// Draws each MujocoJoint in the editor, so it can be seen and clicked:
//
//   hinge   a line along the axis with an arrowhead, and a ring around it
//   slide   a line along the axis with an arrowhead at each end
//   ball    three rings, one about each of the node's axes
//   free    three short lines along the node's axes
//
// It is drawn on top of everything else and sized to reach beyond the geoms on
// the same body, so geometry never hides it.
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

// Installs the editor additions above while the editor is running, and adds
// the command Project > Tools > "Import MJCF...".
//
// The command asks for an MJCF file, imports it with MujocoImporter, saves the
// result as a scene (.tscn), opens that scene, and shows what was left out and
// whether the result matches the original. The scene is an ordinary one made of
// ordinary nodes: nothing ties it to the MJCF file afterwards.
class MujocoEditorPlugin : public godot::EditorPlugin {
    GDCLASS(MujocoEditorPlugin, godot::EditorPlugin);

public:
    void _enter_tree() override;
    void _exit_tree() override;

    // The menu command: shows the file dialog.
    void show_import_dialog();
    // Called when a file has been chosen in the dialog.
    void import_mjcf(const godot::String &p_path);

protected:
    static void _bind_methods() {}

private:
    godot::Ref<MujocoGeomGizmoPlugin> geom_gizmo_plugin;
    godot::Ref<MujocoJointGizmoPlugin> joint_gizmo_plugin;

    godot::EditorFileDialog *file_dialog = nullptr;
    godot::AcceptDialog *result_dialog = nullptr;
    godot::RichTextLabel *result_text = nullptr;

    void show_result(const godot::String &p_bbcode);
};
