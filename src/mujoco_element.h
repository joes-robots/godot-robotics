#pragma once

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <mujoco/mujoco.h>

#include "mujoco_attributes.h"

class MujocoWorld;

// Base class for every node that becomes part of the MuJoCo model: bodies,
// geoms, and later joints, sites and so on.
//
// It holds what all of them share:
//   - finding the MujocoWorld they belong to,
//   - their name in MuJoCo,
//   - telling the world to recompile when something about them changes,
//   - the add_to_spec() hook the world calls to build the model.
//
//   - storage for attributes (see "Attributes" below).
//
// It is abstract: it can't be created on its own, only through a subclass.
//
// Attributes
// ----------
// An element's MuJoCo attributes (a geom's size, density, ...) are not stored as
// one member variable each. The element stores only the attributes that were
// explicitly set on it, in the `overrides` dictionary, the way an MJCF file only
// lists the attributes written on an element. An attribute that isn't in
// `overrides` is inherited: for now from MuJoCo's built-in defaults, later from
// a default class.
//
//   geom.density = 500     -> overrides = { "density": 500.0 }
//   geom.density           -> 500 (the override)
//   geom.friction          -> (1, 0.005, 0.0001) (inherited; not in overrides)
//   geom.density = null    -> removes the override; density is inherited again
//
// "Set to the same value as the default" and "not set" are different things
// here, which is what default classes need. Only `overrides` is saved to scenes.
//
// A subclass describes its attributes with a table (get_attribute_table) and
// says what each one's inherited value is (get_inherited_value). This class
// turns that into inspector properties.
class MujocoElement : public godot::Node3D {
    GDCLASS(MujocoElement, godot::Node3D);

public:
    MujocoElement();

    // The nearest MujocoWorld above this node, or nullptr if there is none.
    MujocoWorld *get_world() const;

    // This element's name in the MuJoCo model: its node path relative to the
    // world, e.g. "Arm/Forearm". Unique, because node paths are. Empty if the
    // node is not under a world.
    godot::String get_mujoco_name() const;

    // Adds this element to p_spec. Called by the world on every compile.
    //
    //   p_body       the MuJoCo body this element belongs to (its nearest
    //                MujocoBody ancestor, or the world body).
    //   p_transform  this node's pose relative to p_body, in Godot's
    //                convention, with any plain Node3D nodes in between
    //                already folded in.
    //
    // Returns the body that this node's own children belong to. A body returns
    // the body it just created. Elements that can't contain other elements
    // return nullptr, and the world doesn't look at their children.
    virtual mjsBody *add_to_spec(mjSpec *p_spec, mjsBody *p_body, const godot::Transform3D &p_transform) const = 0;

    // The transform the world should use for this node when building the model.
    // Normally the node's own transform; see MujocoBody for the exception.
    virtual godot::Transform3D get_spec_transform() const;

    // Attributes. See the class comment.
    bool has_override(const godot::StringName &p_name) const;
    void clear_override(const godot::StringName &p_name);
    godot::Dictionary get_overrides() const;
    void set_overrides(const godot::Dictionary &p_overrides);

protected:
    static void _bind_methods();
    void _notification(int p_what);

    // Asks the world to recompile. Subclasses call this from property setters.
    void request_recompile();

    // True while the simulation, not the user, is moving this node. A move made
    // then is a result of the model, not an edit to it, so it must not recompile.
    bool applying_simulation_pose = false;

    // The subclass's attribute table: a static array and its length. Attributes
    // are referred to by their index in this table.
    virtual const MujocoAttribute *get_attribute_table(int &r_count) const;
    // The value attribute p_index has when it isn't overridden.
    virtual godot::Variant get_inherited_value(int p_index) const;

    // The index of the attribute called p_name, or -1.
    int find_attribute(const godot::StringName &p_name) const;
    // Fetches attribute p_index's override. Returns false if it has none.
    bool get_override(int p_index, godot::Variant &r_value) const;
    // Attribute p_index's value in effect: its override, or else what it inherits.
    godot::Variant get_attribute(int p_index) const;
    // Called after any attribute's value in effect may have changed. For
    // subclasses that keep something in step with their attributes.
    virtual void attributes_changed();

    // Appends this element's attributes as inspector properties. A subclass
    // with attributes calls this from its own _get_property_list. It is not
    // done here in the base class because the inspector files each property
    // under the class whose _get_property_list listed it, and attributes belong
    // under "MujocoGeom" or "MujocoJoint", not "MujocoElement".
    void get_attribute_property_list(godot::List<godot::PropertyInfo> *r_list) const;

    // Godot's dynamic property callbacks, which read and write the attributes.
    bool _get(const godot::StringName &p_name, godot::Variant &r_value) const;
    bool _set(const godot::StringName &p_name, const godot::Variant &p_value);
    bool _property_can_revert(const godot::StringName &p_name) const;
    bool _property_get_revert(const godot::StringName &p_name, godot::Variant &r_value) const;

private:
    // Attribute name -> value, for the attributes explicitly set on this element.
    godot::Dictionary overrides;
};
