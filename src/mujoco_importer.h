#pragma once

#include "mujoco_world.h"

#include <map>
#include <memory>
#include <vector>

#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_string_array.hpp>
#include <mujoco/mujoco.h>

class MujocoDefault;
class MujocoElement;

// Imports an MJCF (MuJoCo XML) model as a tree of MuJoCo nodes.
//
// MuJoCo parses the file itself (default classes, includes, frames, angle units,
// fromto and the other orientation forms), and the model is compiled once. Each
// element becomes a node whose values come from the compiled model, so they are
// exactly what MuJoCo simulates. Default classes become MujocoDefault nodes, and
// each element overrides only what differs from its class.
//
// What can't be represented yet is left out and listed in get_report(). verify()
// compiles the imported nodes and compares the result with the original model.
//
// Meshes (inline, or STL, OBJ and MSH files, which MuJoCo loads) become ArrayMesh
// resources in the file's frame, with the mesh's scale applied. They are part of
// the imported nodes until save_meshes() saves them as files of their own.
//
//     var importer := MujocoImporter.new()
//     var world := importer.import_file("res://robots/arm.xml")
//     print(importer.get_report())
//     print(importer.verify(world))  # [] when the import is faithful
//     add_child(world)
//
// import_to_scene() does all of it and saves the result as a scene; the editor's
// MJCF import commands use it.
class MujocoImporter : public godot::RefCounted {
    GDCLASS(MujocoImporter, godot::RefCounted);

public:
    // A new MujocoWorld (not in the scene tree; every node's owner is the world, so
    // it can be packed into a scene), or null if the model doesn't load (the reason
    // is in get_report()). The world's source_file is p_path.
    MujocoWorld *import_file(const godot::String &p_path);
    MujocoWorld *import_string(const godot::String &p_xml);

    // What the last import left out or changed, one line per kind of thing.
    godot::PackedStringArray get_report() const;

    // Compares p_world (the last import, compiled here if it isn't in the scene
    // tree yet) with the original model: every imported element's
    // compiled values, then p_steps steps of both simulations. Returns the
    // differences found, or an empty array. Things that tumble or land on each
    // other can drift apart from rounding alone (the imported values are 32-bit),
    // so for those the compiled values are what tells.
    godot::PackedStringArray verify(MujocoWorld *p_world, int p_steps = 1000);

    // Saves the last import's meshes into p_directory (created if needed), one
    // .res file per MuJoCo mesh, named after it. A scene packed afterwards refers
    // to these files instead of containing the meshes.
    godot::Error save_meshes(const godot::String &p_directory);
    // Whether the last import made any meshes.
    bool has_meshes() const;

    // Imports p_source and saves it as the scene p_scene_path (a .tscn or .scn),
    // with its meshes in the folder mesh_directory_for_scene(p_scene_path). Unless
    // p_verify_steps is negative, the import is verified first (get_verify_result()).
    // On failure nothing is saved, and get_report() says why.
    godot::Error import_to_scene(const godot::String &p_source, const godot::String &p_scene_path, int p_verify_steps = 1000);
    // What verify() found during the last import_to_scene().
    godot::PackedStringArray get_verify_result() const;
    // Where import_to_scene() saves the meshes of p_scene_path: next to it, in
    // "<scene name>_meshes".
    static godot::String mesh_directory_for_scene(const godot::String &p_scene_path);

protected:
    static void _bind_methods();

private:
    enum Kind {
        KIND_BODY,
        KIND_JOINT,
        KIND_GEOM,
        KIND_SITE,
        KIND_ACTUATOR,
        KIND_SENSOR,
        KIND_TENDON,
        KIND_EQUALITY,
        KIND_COUNT,
    };

    void reset();
    MujocoWorld *import_spec(mjSpec *p_spec);
    void import_classes(MujocoWorld *p_world);
    MujocoDefault *ensure_main_class(MujocoWorld *p_world);
    // Adds the joints, geoms, sites and child bodies of p_body under p_node.
    // p_class is the class in effect there (from the enclosing childclasses).
    void import_body_children(mjsBody *p_body, godot::Node *p_node, MujocoWorld *p_world, const godot::String &p_class);
    void import_tendons(MujocoWorld *p_world);
    void import_equalities(MujocoWorld *p_world);
    void import_actuators(MujocoWorld *p_world);
    void import_sensors(MujocoWorld *p_world);
    void import_excludes(MujocoWorld *p_world);
    void note_unsupported_model_parts();
    // The ArrayMesh for the original model's mesh p_id, made on first use.
    godot::Ref<godot::ArrayMesh> mesh_resource(int p_id);

    // Adds p_element under p_parent, sets its class if it differs from the one it
    // inherits there, and overrides the attributes in p_values that differ from
    // what it inherits.
    void add_element(MujocoElement *p_element, godot::Node *p_parent, const godot::String &p_name, mjsElement *p_spec_element,
            const godot::Dictionary &p_values, MujocoWorld *p_world);
    // The node name for an element: its MJCF name, or p_fallback for unnamed ones.
    static godot::String node_name(mjsElement *p_element, const godot::String &p_fallback);
    // The (node) name of the MuJoCo class an element was created with.
    godot::String class_of(mjsElement *p_element) const;

    // Records one thing left out or changed; get_report() lists each kind once, with a count.
    void note(const godot::String &p_what);

    std::unique_ptr<mjSpec, MjSpecDeleter> spec;
    std::unique_ptr<mjModel, MjModelDeleter> model;
    // MuJoCo class name -> MujocoDefault node name (names are sanitized for nodes).
    std::map<std::string, godot::String> class_names;
    bool main_created = false;
    // For each element of the original model, its node's instance id (0 if left out).
    std::vector<uint64_t> nodes[KIND_COUNT];
    // By MuJoCo mesh id (null where not used).
    std::vector<godot::Ref<godot::ArrayMesh>> meshes;
    godot::PackedStringArray verify_result;
    // Report lines in order, with counts.
    std::vector<std::pair<godot::String, int>> notes;
};
