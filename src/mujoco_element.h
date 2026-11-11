#pragma once

#include "mujoco_attributes.h"

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <mujoco/mujoco.h>

class MujocoDefault;
class MujocoWorld;

// Base class for nodes that become part of a MuJoCo model (bodies, geoms, ...).
// Not instantiable on its own.
//
// An element belongs to its nearest MujocoWorld ancestor; every node in between
// must be a Node3D. Elements hold only Godot-side properties: the world calls
// add_to_spec() on each of them whenever it rebuilds its spec.
//
// Most of an element's properties are class-able attributes (see
// mujoco_attributes.h). The element stores only those set on it, its overrides,
// and inherits the rest from its default class (a MujocoDefault): the class named
// by default_class, else the child_class of the nearest MujocoBody ancestor that
// has one, else "main"; and from that class's parents, then the base defaults.
// In the inspector, an overridden property shows a revert arrow that removes the
// override; scripts do the same by setting the property to null, e.g.
// geom.set("density", null).
class MujocoElement : public godot::Node3D {
    GDCLASS(MujocoElement, godot::Node3D);

public:
    // Adds this element to p_spec.
    //   p_body:      the body this element belongs to (the world body if none).
    //   p_transform: this node's transform relative to p_body, in Godot coordinates.
    //   p_name:      unique MuJoCo name (the node path relative to the world).
    // Returns the body that this node's MuJoCo descendants belong to.
    virtual mjsBody *add_to_spec(mjSpec *p_spec, mjsBody *p_body, const godot::Transform3D &p_transform, const godot::String &p_name);

    // The transform (relative to the parent node) used when building the model.
    virtual godot::Transform3D get_model_transform();

    // Nearest MujocoWorld ancestor, or nullptr.
    MujocoWorld *get_world() const;

    godot::PackedStringArray _get_configuration_warnings() const override;

    // The value of attribute p_key: this element's override if it has one,
    // otherwise the inherited value.
    godot::Variant get_attribute(const godot::StringName &p_key) const;
    bool has_override(const godot::StringName &p_key) const;
    // The overrides, keyed by attribute. This is what scenes save: a property can't
    // save itself, because Godot skips values equal to the class default (an
    // explicit damping = 0 would be lost).
    void set_overrides(const godot::Dictionary &p_overrides);
    godot::Dictionary get_overrides() const;

    // The name of the class to inherit from; empty to use the enclosing bodies'
    // child_class, or "main".
    void set_default_class(const godot::StringName &p_class);
    godot::StringName get_default_class() const;
    // The class this element inherits from, or nullptr for the base defaults.
    MujocoDefault *get_used_class() const;
    // Which section of a default class holds this element type's attributes
    // ("joint", "geom", "site", "tendon" or "actuator"), or nullptr if classes don't apply.
    virtual const char *get_class_section() const { return nullptr; }
    // Writes this element's resolved attributes into the matching section of a
    // MuJoCo default class (used for the elements inside a MujocoDefault).
    virtual void write_class_defaults(mjsDefault *p_default) {}
    // Called when what this element inherits may have changed (a class edited,
    // added, removed or renamed, or a default_class or child_class changed).
    virtual void inherited_changed();
    // The attributes behind inspector property p_property (empty if none). By
    // default, the attribute with the property's name.
    virtual godot::PackedStringArray get_property_attributes(const godot::StringName &p_property) const;

protected:
    static void _bind_methods();
    void _notification(int p_what);

    void _validate_property(godot::PropertyInfo &p_property) const;

    // The class-able attributes of this element type, or nullptr if it has none.
    virtual const MujocoAttributeTable *get_attribute_table() const { return nullptr; }
    // The MuJoCo default class to create this element with: its used class's.
    mjsDefault *get_spec_default(mjSpec *p_spec) const;
    // True for the elements a MujocoDefault keeps for its sections.
    bool is_class_section() const { return class_owner != nullptr; }
    // Stores an override. Reports an error and returns false for an unknown
    // attribute or a value of the wrong type.
    bool set_attribute(const godot::StringName &p_key, const godot::Variant &p_value);
    // Called after overrides change other than through a property setter (revert,
    // set_overrides()), so the element can refresh what depends on them.
    virtual void overrides_changed() {}

    // Revert support: assigning null to an attribute property removes its overrides.
    bool _set(const godot::StringName &p_name, const godot::Variant &p_value);
    bool _property_can_revert(const godot::StringName &p_name) const;
    bool _property_get_revert(const godot::StringName &p_name, godot::Variant &r_property) const;

    // Call whenever something that affects the compiled model changes.
    void request_compile() const;

    // This element's id in its world's current model (running a queued compile
    // first), or -1 if it isn't part of a compiled model.
    int get_model_id(mjtObj p_type);

private:
    friend class MujocoDefault;

    // The value attribute p_attribute has when this element doesn't override it.
    godot::Variant get_inherited(const MujocoAttribute &p_attribute) const;

    godot::Dictionary overrides;
    godot::StringName default_class;
    // Set on the elements a MujocoDefault keeps for its sections: they inherit
    // from that class's parent, and report changes to the class.
    MujocoDefault *class_owner = nullptr;
    // Cache for get_model_id(), valid while the world's model generation matches.
    uint64_t cached_generation = 0;
    int cached_id = -1;
};
