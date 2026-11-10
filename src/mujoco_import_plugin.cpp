#include "mujoco_import_plugin.h"
#include "mujoco_importer.h"
#include "mujoco_world.h"

#include <godot_cpp/classes/accept_dialog.hpp>
#include <godot_cpp/classes/button.hpp>
#include <godot_cpp/classes/confirmation_dialog.hpp>
#include <godot_cpp/classes/dir_access.hpp>
#include <godot_cpp/classes/editor_file_dialog.hpp>
#include <godot_cpp/classes/editor_file_system.hpp>
#include <godot_cpp/classes/editor_interface.hpp>
#include <godot_cpp/classes/editor_undo_redo_manager.hpp>
#include <godot_cpp/classes/file_access.hpp>
#include <godot_cpp/classes/h_box_container.hpp>
#include <godot_cpp/classes/project_settings.hpp>
#include <godot_cpp/classes/rich_text_label.hpp>
#include <godot_cpp/classes/theme.hpp>
#include <godot_cpp/classes/undo_redo.hpp>
#include <godot_cpp/variant/callable_method_pointer.hpp>

using namespace godot;

namespace {

// Report lines are plain text inside BBCode.
String escape_bbcode(const String &p_text)
{
    return p_text.replace("[", "[lb]");
}

// A BBCode bulleted list (it wraps long lines with a hanging indent).
String bullet_list(const PackedStringArray &p_lines)
{
    String text = "[ul]";
    for (const String &line : p_lines) {
        text += escape_bbcode(line) + "\n";
    }
    return text + "[/ul]\n";
}

// p_node and its descendants (not internal ones), each with p_owner, or with its
// current owner if p_owner is null.
// Clears the owner of p_node and its descendants (not internal ones).
void clear_owners(Node *p_node)
{
    p_node->set_owner(nullptr);
    for (int i = 0; i < p_node->get_child_count(); i++) {
        clear_owners(p_node->get_child(i));
    }
}

void collect_owners(Node *p_node, Node *p_owner, Dictionary &r_owners)
{
    r_owners[p_node] = p_owner ? p_owner : p_node->get_owner();
    for (int i = 0; i < p_node->get_child_count(); i++) {
        collect_owners(p_node->get_child(i), p_owner, r_owners);
    }
}

} // namespace

// --- FileSystem dock ----------------------------------------------------------

void MujocoFileContextMenu::_popup_menu(const PackedStringArray &p_paths)
{
    if (p_paths.size() != 1 || !MujocoImportPlugin::is_mjcf_file(p_paths[0])) {
        return;
    }
    const Ref<Theme> theme = EditorInterface::get_singleton()->get_editor_theme();
    add_context_menu_item("Convert to MuJoCo Scene", callable_mp(plugin, &MujocoImportPlugin::convert_files),
            theme.is_valid() ? theme->get_icon("PackedScene", "EditorIcons") : Ref<Texture2D>());
}

// --- Inspector ----------------------------------------------------------------

bool MujocoWorldInspector::_can_handle(Object *p_object) const
{
    return Object::cast_to<MujocoWorld>(p_object) != nullptr;
}

void MujocoWorldInspector::_parse_begin(Object *p_object)
{
    const MujocoWorld *world = Object::cast_to<MujocoWorld>(p_object);
    const uint64_t id = world->get_instance_id();

    HBoxContainer *box = memnew(HBoxContainer);
    Button *load = memnew(Button);
    load->set_text("Load MJCF...");
    load->set_tooltip_text("Replace everything in this world with a model imported from an MJCF (MuJoCo XML) file.");
    load->set_h_size_flags(Control::SIZE_EXPAND_FILL);
    load->connect("pressed", callable_mp(plugin, &MujocoImportPlugin::load_pressed).bind(id));
    box->add_child(load);

    Button *reload = memnew(Button);
    reload->set_text("Reload from Source");
    const String source = world->get_source_file();
    reload->set_disabled(source.is_empty());
    reload->set_tooltip_text(source.is_empty() ? String("This world wasn't imported from a file (see source_file).")
                                               : "Import " + source + " again, replacing everything in this world.");
    reload->set_h_size_flags(Control::SIZE_EXPAND_FILL);
    reload->connect("pressed", callable_mp(plugin, &MujocoImportPlugin::reload_pressed).bind(id));
    box->add_child(reload);

    add_custom_control(box);
}

// --- Plugin -------------------------------------------------------------------

bool MujocoImportPlugin::is_mjcf_file(const String &p_path)
{
    if (p_path.get_extension().to_lower() != "xml") {
        return false;
    }
    const Ref<FileAccess> file = FileAccess::open(p_path, FileAccess::READ);
    if (file.is_null()) {
        return false;
    }
    // The root element comes after at most a license comment or two.
    const PackedByteArray head = file->get_buffer(16384);
    return head.get_string_from_utf8().contains("<mujoco");
}

void MujocoImportPlugin::_enter_tree()
{
    Control *base = EditorInterface::get_singleton()->get_base_control();
    const float scale = EditorInterface::get_singleton()->get_editor_scale();

    source_dialog = memnew(EditorFileDialog);
    source_dialog->set_access(EditorFileDialog::ACCESS_FILESYSTEM);
    source_dialog->set_file_mode(EditorFileDialog::FILE_MODE_OPEN_FILE);
    source_dialog->add_filter("*.xml", "MJCF (MuJoCo XML)");
    source_dialog->set_title("Open an MJCF File");
    source_dialog->connect("file_selected", callable_mp(this, &MujocoImportPlugin::source_selected));
    base->add_child(source_dialog);

    scene_dialog = memnew(EditorFileDialog);
    scene_dialog->set_access(EditorFileDialog::ACCESS_RESOURCES);
    scene_dialog->set_file_mode(EditorFileDialog::FILE_MODE_SAVE_FILE);
    scene_dialog->add_filter("*.tscn", "Text Scene");
    scene_dialog->add_filter("*.scn", "Binary Scene");
    scene_dialog->set_title("Save the MuJoCo Scene As");
    scene_dialog->connect("file_selected", callable_mp(this, &MujocoImportPlugin::scene_selected));
    base->add_child(scene_dialog);

    confirm_dialog = memnew(ConfirmationDialog);
    confirm_dialog->set_title("Import MJCF");
    confirm_dialog->connect("confirmed", callable_mp(this, &MujocoImportPlugin::confirmed));
    base->add_child(confirm_dialog);

    result_dialog = memnew(AcceptDialog);
    result_dialog->set_title("MJCF Import");
    result_text = memnew(RichTextLabel);
    result_text->set_use_bbcode(true);
    result_text->set_selection_enabled(true);
    result_text->set_context_menu_enabled(true);
    result_text->set_custom_minimum_size(Vector2(640, 360) * scale);
    result_dialog->add_child(result_text);
    open_button = result_dialog->add_button("Open Scene", true, "open_scene");
    result_dialog->connect("custom_action", callable_mp(this, &MujocoImportPlugin::result_action));
    base->add_child(result_dialog);

    add_tool_menu_item("Import MJCF...", callable_mp(this, &MujocoImportPlugin::import_menu_pressed));
    context_menu.instantiate();
    context_menu->plugin = this;
    add_context_menu_plugin(EditorContextMenuPlugin::CONTEXT_SLOT_FILESYSTEM, context_menu);
    inspector.instantiate();
    inspector->plugin = this;
    add_inspector_plugin(inspector);
}

void MujocoImportPlugin::_exit_tree()
{
    remove_inspector_plugin(inspector);
    inspector.unref();
    remove_context_menu_plugin(context_menu);
    context_menu.unref();
    remove_tool_menu_item("Import MJCF...");
    for (Node *dialog : { static_cast<Node *>(source_dialog), static_cast<Node *>(scene_dialog), static_cast<Node *>(confirm_dialog), static_cast<Node *>(result_dialog) }) {
        dialog->queue_free();
    }
    source_dialog = nullptr;
    scene_dialog = nullptr;
    confirm_dialog = nullptr;
    result_dialog = nullptr;
    result_text = nullptr;
    open_button = nullptr;
}

void MujocoImportPlugin::import_menu_pressed()
{
    pending = PENDING_SCENE;
    source_dialog->popup_file_dialog();
}

void MujocoImportPlugin::convert_files(const Variant &p_paths)
{
    const PackedStringArray paths = p_paths;
    if (paths.size() != 1) {
        return;
    }
    pending_source = paths[0];
    pending_scene = pending_source.get_basename() + ".tscn";
    if (FileAccess::file_exists(pending_scene)) {
        pending = PENDING_CONVERT;
        ask(vformat("%s already exists. Replace it with a new import of %s?", pending_scene, pending_source.get_file()), "Replace");
        return;
    }
    convert(pending_source, pending_scene);
}

void MujocoImportPlugin::load_pressed(uint64_t p_world_id)
{
    pending = PENDING_LOAD;
    pending_world = p_world_id;
    const MujocoWorld *world = pending_world_node();
    if (world && !world->get_source_file().is_empty()) {
        source_dialog->set_current_path(ProjectSettings::get_singleton()->globalize_path(world->get_source_file()));
    }
    source_dialog->popup_file_dialog();
}

void MujocoImportPlugin::reload_pressed(uint64_t p_world_id)
{
    pending = PENDING_LOAD;
    pending_world = p_world_id;
    const MujocoWorld *world = pending_world_node();
    if (!world) {
        return;
    }
    pending_source = world->get_source_file();
    ask(vformat("Import %s again into \"%s\"?\n\nThis replaces everything in the world, including changes made since the last import. You can undo it.",
                pending_source, world->get_name()),
            "Reload");
}

MujocoWorld *MujocoImportPlugin::pending_world_node() const
{
    return Object::cast_to<MujocoWorld>(ObjectDB::get_instance(pending_world));
}

void MujocoImportPlugin::ask(const String &p_text, const String &p_ok)
{
    confirm_dialog->set_text(p_text);
    confirm_dialog->set_ok_button_text(p_ok);
    confirm_dialog->popup_centered();
}

void MujocoImportPlugin::source_selected(const String &p_path)
{
    pending_source = p_path;
    if (pending == PENDING_SCENE) {
        // Suggest a scene named after the file, in the project's root.
        scene_dialog->set_current_path("res://" + p_path.get_file().get_basename() + ".tscn");
        scene_dialog->popup_file_dialog();
        return;
    }
    if (pending == PENDING_LOAD) {
        MujocoWorld *world = pending_world_node();
        if (!world) {
            return;
        }
        if (world->get_child_count() > 0) {
            ask(vformat("Replace everything in \"%s\" with a model imported from %s?\n\nYou can undo this.", world->get_name(), p_path.get_file()), "Replace");
            return;
        }
        load_into_world(world, p_path);
    }
}

void MujocoImportPlugin::scene_selected(const String &p_path)
{
    // The file dialog has already asked about replacing an existing scene.
    convert(pending_source, p_path);
}

void MujocoImportPlugin::confirmed()
{
    switch (pending) {
        case PENDING_CONVERT:
            convert(pending_source, pending_scene);
            break;
        case PENDING_LOAD:
            if (MujocoWorld *world = pending_world_node()) {
                load_into_world(world, pending_source);
            }
            break;
        default:
            break;
    }
}

void MujocoImportPlugin::convert(const String &p_source, const String &p_scene)
{
    pending = PENDING_NONE;
    Ref<MujocoImporter> importer;
    importer.instantiate();
    const Error error = importer->import_to_scene(p_source, p_scene);
    EditorInterface *editor = EditorInterface::get_singleton();
    editor->get_resource_filesystem()->scan();
    if (error != OK) {
        show_result("[b]The import failed.[/b]", importer.ptr(), PackedStringArray(), true);
        return;
    }
    // A scene that is open in the editor would otherwise keep showing the old version.
    if (editor->get_open_scenes().has(p_scene)) {
        editor->reload_scene_from_path(p_scene);
    }
    String summary = vformat("Imported [b]%s[/b] as [b]%s[/b].", escape_bbcode(p_source.get_file()), escape_bbcode(p_scene));
    if (importer->has_meshes()) {
        summary += vformat(" Its meshes are in [b]%s[/b].", escape_bbcode(MujocoImporter::mesh_directory_for_scene(p_scene)));
    }
    result_scene = p_scene;
    show_result(summary, importer.ptr(), importer->get_verify_result(), false);
}

void MujocoImportPlugin::load_into_world(MujocoWorld *p_world, const String &p_source)
{
    pending = PENDING_NONE;
    result_scene = String();
    EditorInterface *editor = EditorInterface::get_singleton();
    Node *root = editor->get_edited_scene_root();
    ERR_FAIL_COND_MSG(!root || (root != p_world && !root->is_ancestor_of(p_world)), "The world isn't in the edited scene.");

    Ref<MujocoImporter> importer;
    importer.instantiate();
    MujocoWorld *imported = importer->import_file(p_source);
    if (!imported) {
        show_result("[b]The import failed.[/b]", importer.ptr(), PackedStringArray(), true);
        return;
    }
    String summary = vformat("Imported [b]%s[/b] into [b]%s[/b].", escape_bbcode(p_source.get_file()), escape_bbcode(p_world->get_name()));
    if (importer->has_meshes()) {
        const String scene_path = root->get_scene_file_path();
        if (scene_path.is_empty()) {
            summary += " The scene isn't saved yet, so its meshes are stored inside it.";
        } else {
            const String directory = MujocoImporter::mesh_directory_for_scene(scene_path);
            if (importer->save_meshes(directory) == OK) {
                summary += vformat(" Its meshes are in [b]%s[/b].", escape_bbcode(directory));
            } else {
                summary += " The meshes could not be saved as files, so they are stored inside the scene.";
            }
            editor->get_resource_filesystem()->scan();
        }
    }
    const PackedStringArray verify = importer->verify(imported);

    // Swap the children, keeping both sets alive for undo and redo.
    Array old_children;
    Array new_children;
    Dictionary old_owners;
    Dictionary new_owners;
    for (int i = 0; i < p_world->get_child_count(); i++) {
        Node *child = p_world->get_child(i);
        old_children.push_back(child);
        collect_owners(child, nullptr, old_owners);
    }
    while (imported->get_child_count() > 0) {
        Node *child = imported->get_child(0);
        imported->remove_child(child);
        // Their owner was the imported world; the scene's root takes over when they're added.
        clear_owners(child);
        new_children.push_back(child);
        collect_owners(child, root, new_owners);
    }

    EditorUndoRedoManager *undo_redo = get_undo_redo();
    undo_redo->create_action(vformat("Load MJCF into %s", p_world->get_name()), UndoRedo::MERGE_DISABLE, p_world);
    undo_redo->add_do_method(this, "_replace_children", p_world, old_children, new_children, new_owners);
    undo_redo->add_undo_method(this, "_replace_children", p_world, new_children, old_children, old_owners);
    // The physics options and the source come with the model.
    const TypedArray<Dictionary> properties = imported->get_property_list();
    for (int64_t i = 0; i < properties.size(); i++) {
        const String name = Dictionary(properties[i])["name"];
        if (name.begins_with("option_") || name == "source_file") {
            undo_redo->add_do_property(p_world, name, imported->get(name));
            undo_redo->add_undo_property(p_world, name, p_world->get(name));
        }
    }
    for (int64_t i = 0; i < new_children.size(); i++) {
        undo_redo->add_do_reference(Object::cast_to<Object>(new_children[i]));
    }
    for (int64_t i = 0; i < old_children.size(); i++) {
        undo_redo->add_undo_reference(Object::cast_to<Object>(old_children[i]));
    }
    undo_redo->commit_action();
    memdelete(imported);

    show_result(summary, importer.ptr(), verify, false);
}

void MujocoImportPlugin::replace_children(Node *p_world, const Array &p_remove, const Array &p_add, const Dictionary &p_owners)
{
    for (int64_t i = 0; i < p_remove.size(); i++) {
        if (Node *child = Object::cast_to<Node>(p_remove[i])) {
            p_world->remove_child(child);
        }
    }
    for (int64_t i = 0; i < p_add.size(); i++) {
        if (Node *child = Object::cast_to<Node>(p_add[i])) {
            p_world->add_child(child);
        }
    }
    // Owners need the nodes in the tree; they decide what is saved with the scene.
    const Array nodes = p_owners.keys();
    for (int64_t i = 0; i < nodes.size(); i++) {
        Node *node = Object::cast_to<Node>(nodes[i]);
        Node *owner = Object::cast_to<Node>(p_owners[nodes[i]]);
        if (node && owner) {
            node->set_owner(owner);
        }
    }
    // The inspector's buttons depend on source_file.
    p_world->notify_property_list_changed();
}

void MujocoImportPlugin::show_result(const String &p_summary, const MujocoImporter *p_importer, const PackedStringArray &p_verify, bool p_failed)
{
    String text = p_summary + String("\n\n");
    const PackedStringArray report = p_importer->get_report();
    if (p_failed) {
        text += bullet_list(report);
    } else {
        text += "[b]Left out or changed[/b]\n";
        text += report.is_empty() ? String("  Nothing: everything in the file was imported.\n") : bullet_list(report);
        text += "\n[b]Check[/b]\n";
        if (p_verify.is_empty()) {
            text += "  The imported nodes compile to the same MuJoCo model, and both simulate the same for 1000 steps.\n";
        } else {
            text += "  Compared with the original model (expected where something was left out):\n" + bullet_list(p_verify);
        }
    }
    result_text->clear();
    result_text->append_text(text);
    open_button->set_visible(!result_scene.is_empty());
    result_dialog->popup_centered();
}

void MujocoImportPlugin::result_action(const StringName &p_action)
{
    if (p_action == StringName("open_scene") && !result_scene.is_empty()) {
        EditorInterface::get_singleton()->open_scene_from_path(result_scene);
        result_dialog->hide();
    }
}

void MujocoImportPlugin::_bind_methods()
{
    // Called by the undo/redo history.
    ClassDB::bind_method(D_METHOD("_replace_children", "world", "remove", "add", "owners"), &MujocoImportPlugin::replace_children);
}
