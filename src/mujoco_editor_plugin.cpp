#include "mujoco_editor_plugin.h"

#include <godot_cpp/classes/accept_dialog.hpp>
#include <godot_cpp/classes/editor_file_dialog.hpp>
#include <godot_cpp/classes/editor_file_system.hpp>
#include <godot_cpp/classes/editor_interface.hpp>
#include <godot_cpp/classes/file_access.hpp>
#include <godot_cpp/classes/mesh.hpp>
#include <godot_cpp/classes/packed_scene.hpp>
#include <godot_cpp/classes/project_settings.hpp>
#include <godot_cpp/classes/resource_saver.hpp>
#include <godot_cpp/classes/rich_text_label.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/classes/triangle_mesh.hpp>

#include "mujoco_geom.h"
#include "mujoco_importer.h"
#include "mujoco_joint.h"
#include "mujoco_world.h"

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

// ---- The plugin, and the import command ----

namespace {

const char *const IMPORT_COMMAND = "Import MJCF...";

// Text to be shown in a RichTextLabel must not be mistaken for its markup.
String escape_bbcode(const String &p_text)
{
    return p_text.replace("[", "[lb]");
}

// A scene path that doesn't exist yet: the wanted one, or with _2, _3... added.
// An earlier import may have been edited since, so it is never overwritten.
String unused_scene_path(const String &p_directory, const String &p_name)
{
    String path = p_directory.path_join(p_name + String(".tscn"));
    for (int i = 2; FileAccess::file_exists(path); i++) {
        path = p_directory.path_join(p_name + String("_") + itos(i) + String(".tscn"));
    }
    return path;
}

} // namespace

void MujocoEditorPlugin::_enter_tree()
{
    geom_gizmo_plugin.instantiate();
    add_node_3d_gizmo_plugin(geom_gizmo_plugin);
    joint_gizmo_plugin.instantiate();
    add_node_3d_gizmo_plugin(joint_gizmo_plugin);

    // The file chooser. MJCF files are often kept outside the project, so it
    // browses the whole file system.
    file_dialog = memnew(EditorFileDialog);
    file_dialog->set_file_mode(FileDialog::FILE_MODE_OPEN_FILE);
    file_dialog->set_access(FileDialog::ACCESS_FILESYSTEM);
    file_dialog->add_filter("*.xml", "MJCF files");
    file_dialog->set_title("Import MJCF");
    file_dialog->connect("file_selected", callable_mp(this, &MujocoEditorPlugin::import_mjcf));
    EditorInterface::get_singleton()->get_base_control()->add_child(file_dialog);

    // The window that shows the outcome.
    result_dialog = memnew(AcceptDialog);
    result_dialog->set_title("MJCF Import");
    result_text = memnew(RichTextLabel);
    result_text->set_use_bbcode(true);
    result_text->set_custom_minimum_size(Vector2(640, 360));
    result_text->set_selection_enabled(true);
    result_dialog->add_child(result_text);
    EditorInterface::get_singleton()->get_base_control()->add_child(result_dialog);

    add_tool_menu_item(IMPORT_COMMAND, callable_mp(this, &MujocoEditorPlugin::show_import_dialog));
}

void MujocoEditorPlugin::_exit_tree()
{
    remove_tool_menu_item(IMPORT_COMMAND);
    if (file_dialog) {
        file_dialog->queue_free();
        file_dialog = nullptr;
    }
    if (result_dialog) {
        result_dialog->queue_free();
        result_dialog = nullptr;
        result_text = nullptr;
    }

    remove_node_3d_gizmo_plugin(joint_gizmo_plugin);
    joint_gizmo_plugin.unref();
    remove_node_3d_gizmo_plugin(geom_gizmo_plugin);
    geom_gizmo_plugin.unref();
}

void MujocoEditorPlugin::show_import_dialog()
{
    file_dialog->popup_file_dialog();
}

void MujocoEditorPlugin::show_result(const String &p_bbcode)
{
    result_text->set_text(p_bbcode);
    result_dialog->popup_centered();
}

void MujocoEditorPlugin::import_mjcf(const String &p_path)
{
    const String file_name = escape_bbcode(p_path.get_file());

    Ref<MujocoImporter> importer;
    importer.instantiate();
    MujocoWorld *world = importer->import_file(p_path);
    if (!world) {
        show_result(String("[b]Could not import ") + file_name + String("[/b]\n\n") + escape_bbcode(importer->get_error()));
        return;
    }

    // Check the result against the original. That needs a compiled model, and
    // a world compiles when it enters the scene tree, so it is put in the tree
    // for a moment and taken out again before anything is drawn.
    add_child(world);
    const Array differences = importer->verify_file(world, p_path);
    remove_child(world);

    // Save beside the MJCF file if that is inside the project, else at the
    // project's top level.
    const String local_path = ProjectSettings::get_singleton()->localize_path(p_path);
    const String directory = local_path.begins_with("res://") ? local_path.get_base_dir() : String("res://");
    const String scene_path = unused_scene_path(directory, p_path.get_file().get_basename());

    Ref<PackedScene> scene;
    scene.instantiate();
    Error result = scene->pack(world);
    if (result == OK) {
        result = ResourceSaver::get_singleton()->save(scene, scene_path);
    }
    memdelete(world);
    if (result != OK) {
        show_result(String("[b]Could not save ") + escape_bbcode(scene_path) + String("[/b]\n\nError ") + itos(result) + String("."));
        return;
    }

    // Make the editor notice the new file, and open it.
    EditorInterface::get_singleton()->get_resource_filesystem()->scan();
    EditorInterface::get_singleton()->open_scene_from_path(scene_path);

    // The outcome. [ul] makes a bulleted list, one item per line.
    String text = String("[b]Imported ") + file_name + String("[/b]\nSaved as ") + escape_bbcode(scene_path) + String("\n\n");

    const Array report = importer->get_report();
    if (report.is_empty()) {
        text += "Everything in the file was imported.\n\n";
    } else {
        text += "[b]Left out or changed[/b]\n[ul]\n";
        for (int i = 0; i < report.size(); i++) {
            const Dictionary entry = report[i];
            const String element = entry["element"];
            text += (element.is_empty() ? String() : String("[i]") + escape_bbcode(element) + String("[/i]: "))
                    + escape_bbcode(entry["message"]) + String("\n");
        }
        text += "[/ul]\n\n";
    }

    if (differences.is_empty()) {
        text += "[b]Compared with the original[/b]\nThe imported model matches the original: every compiled value agrees, and both behave the same over 200 simulated steps.";
    } else {
        text += "[b]Compared with the original[/b]\nThe compiled models differ in these fields:\n[ul]\n";
        for (int i = 0; i < differences.size(); i++) {
            const Dictionary entry = differences[i];
            text += String("[code]") + escape_bbcode(entry["field"]) + String("[/code]: ") + escape_bbcode(entry["message"]) + String("\n");
        }
        text += "[/ul]";
    }
    show_result(text);
}
