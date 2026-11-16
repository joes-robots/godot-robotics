#pragma once

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/string.hpp>
#include <mujoco/mujoco.h>

class MujocoWorld;
class MujocoElement;

// Reads an MJCF file (MuJoCo's XML format) into the same nodes you would build
// by hand: a MujocoWorld with MujocoBody, MujocoGeom and MujocoJoint nodes
// below it.
//
//   var importer := MujocoImporter.new()
//   var world := importer.import_file("res://robot.xml")
//   if world == null:
//       print(importer.get_error())
//   print(importer.get_report_text())
//
// How it works. MuJoCo parses the file into a spec, and compiles the spec into
// a model. The importer walks the spec for the structure (which bodies, geoms
// and joints there are, how they nest, what they are called) and takes the
// numbers from the compiled model, where MuJoCo has already resolved every
// alternative way MJCF has of writing them: angles in degrees, orientations as
// euler angles, capsules given by their two ends, poses nested in frames.
//
// What it leaves out. Anything there is no node or attribute for is skipped,
// never approximated, and listed in the report.
class MujocoImporter : public godot::RefCounted {
    GDCLASS(MujocoImporter, godot::RefCounted);

public:
    // Both return a new MujocoWorld that is not in the scene tree, or null on
    // failure (see get_error). The caller owns the returned node.
    MujocoWorld *import_file(const godot::String &p_path);
    MujocoWorld *import_string(const godot::String &p_xml);

    // Why the last import failed; empty if it succeeded.
    godot::String get_error() const;

    // What the last import left out or changed. Each entry is a Dictionary:
    //   kind     "skipped_element", "skipped_attribute", "flattened_default" or "note"
    //   element  the name of the element concerned, or "" for the whole file
    //   message  what happened, in words
    godot::Array get_report() const;
    // The same, as text: one line per entry.
    godot::String get_report_text() const;

    // Checks an imported world against the MJCF it came from: compiles the
    // original, and compares it with p_world's compiled model field by field
    // and by simulating both (see mujoco_verify.h for exactly what is
    // compared). p_world must be in the scene tree and compiled.
    //
    // Returns one Dictionary per difference, with "field", "message", "count"
    // and "max_difference"; an empty array means the models match. If the
    // check itself can't be made, the single entry has the field "error".
    // Does not affect get_error() or get_report().
    godot::Array verify_file(MujocoWorld *p_world, const godot::String &p_path);
    godot::Array verify_string(MujocoWorld *p_world, const godot::String &p_xml);
    // Differences as text, one line each; "No differences." for an empty array.
    static godot::String differences_to_text(const godot::Array &p_differences);

protected:
    static void _bind_methods();

private:
    godot::Array verify(MujocoWorld *p_world, mjSpec *p_spec, const godot::String &p_parse_error);

    godot::String error;
    godot::Array report;

    MujocoWorld *build(mjSpec *p_spec, const godot::String &p_fallback_name);
    void import_options(MujocoWorld *p_world, const mjModel *p_model);
    void import_children(godot::Node *p_parent, MujocoWorld *p_world, mjsBody *p_body, const mjModel *p_model);
    void import_body(godot::Node *p_parent, MujocoWorld *p_world, mjsBody *p_body, const mjModel *p_model);
    void import_geom(godot::Node *p_parent, MujocoWorld *p_world, mjsGeom *p_geom, const mjModel *p_model);
    void import_joint(godot::Node *p_parent, MujocoWorld *p_world, mjsJoint *p_joint, const mjModel *p_model);
    void report_other_elements(mjSpec *p_spec);

    // Adds p_node under p_parent, named after p_element (or p_default_name if
    // it has no name), and makes it keep its exact MJCF name.
    void add_node(godot::Node *p_parent, MujocoWorld *p_world, MujocoElement *p_node, mjsElement *p_element, const godot::String &p_default_name);
    // Sets an attribute if p_value differs from what the node has anyway.
    static void set_if_different(MujocoElement *p_node, const godot::String &p_attribute, const godot::Variant &p_value);
    void add_report(const godot::String &p_kind, const godot::String &p_element, const godot::String &p_message);
};
