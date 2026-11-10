#pragma once

// Editor commands for importing MJCF (MuJoCo XML) files. Editor-only, like
// mujoco_editor_plugin.h.
//
// - Project > Tools > Import MJCF...: pick a file anywhere, then where to save
//   the scene.
// - FileSystem dock, right-click an MJCF file: Convert to MuJoCo Scene (saved
//   next to it, with the same name).
// - MujocoWorld inspector: Load MJCF... replaces the world's contents with an
//   import, Reload from Source imports its source_file again. Both can be undone.
//
// Each shows what was left out and what MujocoImporter.verify() found. Meshes are
// saved next to the scene, in "<scene name>_meshes".

#include <godot_cpp/classes/editor_context_menu_plugin.hpp>
#include <godot_cpp/classes/editor_inspector_plugin.hpp>
#include <godot_cpp/classes/editor_plugin.hpp>

namespace godot {
class AcceptDialog;
class Button;
class ConfirmationDialog;
class EditorFileDialog;
class RichTextLabel;
} // namespace godot

class MujocoImporter;
class MujocoImportPlugin;
class MujocoWorld;

// "Convert to MuJoCo Scene" in the FileSystem dock's menu for an MJCF file.
class MujocoFileContextMenu : public godot::EditorContextMenuPlugin {
    GDCLASS(MujocoFileContextMenu, godot::EditorContextMenuPlugin);

public:
    void _popup_menu(const godot::PackedStringArray &p_paths) override;

    MujocoImportPlugin *plugin = nullptr;

protected:
    static void _bind_methods() {}
};

// "Load MJCF..." and "Reload from Source" buttons at the top of a MujocoWorld's inspector.
class MujocoWorldInspector : public godot::EditorInspectorPlugin {
    GDCLASS(MujocoWorldInspector, godot::EditorInspectorPlugin);

public:
    bool _can_handle(godot::Object *p_object) const override;
    void _parse_begin(godot::Object *p_object) override;

    MujocoImportPlugin *plugin = nullptr;

protected:
    static void _bind_methods() {}
};

class MujocoImportPlugin : public godot::EditorPlugin {
    GDCLASS(MujocoImportPlugin, godot::EditorPlugin);

public:
    void _enter_tree() override;
    void _exit_tree() override;

    // Whether p_path is an .xml file whose root element is <mujoco>.
    static bool is_mjcf_file(const godot::String &p_path);

    // The commands (see the file comment).
    void import_menu_pressed();
    void convert_files(const godot::Variant &p_paths);
    void load_pressed(uint64_t p_world_id);
    void reload_pressed(uint64_t p_world_id);

protected:
    static void _bind_methods();

private:
    enum Pending {
        PENDING_NONE,
        PENDING_SCENE, // the Tools menu: a source was picked, the scene comes next
        PENDING_CONVERT, // convert pending_source to pending_scene once confirmed
        PENDING_LOAD, // load pending_source into pending_world once confirmed (or picked)
    };

    void source_selected(const godot::String &p_path);
    void scene_selected(const godot::String &p_path);
    void confirmed();
    void result_action(const godot::StringName &p_action);
    void ask(const godot::String &p_text, const godot::String &p_ok);

    void convert(const godot::String &p_source, const godot::String &p_scene);
    void load_into_world(MujocoWorld *p_world, const godot::String &p_source);
    // Undo/redo step of load_into_world(): swaps the world's children and sets owners.
    void replace_children(godot::Node *p_world, const godot::Array &p_remove, const godot::Array &p_add, const godot::Dictionary &p_owners);
    void show_result(const godot::String &p_summary, const MujocoImporter *p_importer, const godot::PackedStringArray &p_verify, bool p_failed);
    MujocoWorld *pending_world_node() const;

    godot::EditorFileDialog *source_dialog = nullptr;
    godot::EditorFileDialog *scene_dialog = nullptr;
    godot::ConfirmationDialog *confirm_dialog = nullptr;
    godot::AcceptDialog *result_dialog = nullptr;
    godot::RichTextLabel *result_text = nullptr;
    godot::Button *open_button = nullptr;
    godot::Ref<MujocoFileContextMenu> context_menu;
    godot::Ref<MujocoWorldInspector> inspector;

    Pending pending = PENDING_NONE;
    godot::String pending_source;
    godot::String pending_scene;
    uint64_t pending_world = 0;
    // The scene the result dialog's "Open Scene" button opens.
    godot::String result_scene;
};
