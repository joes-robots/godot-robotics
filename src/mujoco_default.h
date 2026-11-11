#pragma once

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/templates/list.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <mujoco/mujoco.h>

class MujocoElement;
class MujocoWorld;

// A MuJoCo default class: attribute values that joints, geoms, sites, tendons
// and actuators inherit when they don't set them themselves (MJCF's <default>).
//
// Classes are MujocoDefault nodes under a MujocoWorld; the node name is the class
// name. Nesting them makes a class tree: a nested class inherits from the class
// it is in. A top-level class named "main" is MuJoCo's root class: every other
// top-level class inherits from it, and elements without a class use it.
//
// The inspector shows a section per element type ("Joint", "Geom", ...) with the
// same properties as the elements. Each is inherited from the parent class or
// overridden here, and reverting removes the override, as on elements. Scripts
// use the section as a prefix: default_class.set("geom/density", 500).
//
// Frames (positions, orientations, joint axes) always come from the elements'
// own transforms, so classes don't have them.
class MujocoDefault : public godot::Node {
    GDCLASS(MujocoDefault, godot::Node);

public:
    MujocoDefault();
    ~MujocoDefault();

    godot::PackedStringArray _get_configuration_warnings() const override;

    // The world this class belongs to: the nearest MujocoWorld ancestor.
    MujocoWorld *get_world() const;
    // MuJoCo's root class: top-level and named "main".
    bool is_main() const;
    // The class this one inherits from, or nullptr for the base defaults.
    MujocoDefault *get_parent_class() const;

    // The element holding this class's values for one section ("joint", "geom",
    // "site", "tendon" or "actuator"), or nullptr. Its overrides are this class's overrides.
    MujocoElement *get_section_element(const char *p_section) const;
    // Writes this class's resolved values into p_default.
    void write_to_spec(mjsDefault *p_default) const;

    // Overrides of all sections, as {section: {attribute: value}}. Scenes save this.
    void set_overrides(const godot::Dictionary &p_overrides);
    godot::Dictionary get_overrides() const;

    // For the section elements: one of them changed.
    void section_changed();

protected:
    static void _bind_methods();
    void _notification(int p_what);
    bool _set(const godot::StringName &p_name, const godot::Variant &p_value);
    bool _get(const godot::StringName &p_name, godot::Variant &r_ret) const;
    void _get_property_list(godot::List<godot::PropertyInfo> *p_list) const;
    bool _property_can_revert(const godot::StringName &p_name) const;
    bool _property_get_revert(const godot::StringName &p_name, godot::Variant &r_property) const;

private:
    // Splits "geom/density" into the geom section element and "density". Returns
    // nullptr if p_name isn't one of this class's properties.
    MujocoElement *split_property(const godot::StringName &p_name, godot::StringName &r_property) const;

    static constexpr int SECTION_COUNT = 5;
    // Not in the scene tree; owned by this node.
    MujocoElement *sections[SECTION_COUNT] = {};
};
