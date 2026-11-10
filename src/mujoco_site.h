#pragma once

#include "mujoco_element.h"
#include "mujoco_geom.h"

// A MuJoCo site: a frame attached to the body it belongs to (the nearest
// MujocoBody ancestor, or the world body), with a shape that has no mass and no
// collisions. Sensors measure at sites; a touch sensor counts the contacts
// inside its site's shape.
//
// Shapes and sizes work like MujocoGeom's (no planes), including how the size is
// stored. Sites are only drawn by the editor gizmo, not in the running game.
class MujocoSite : public MujocoElement {
    GDCLASS(MujocoSite, MujocoElement);

public:
    mjsBody *add_to_spec(mjSpec *p_spec, mjsBody *p_body, const godot::Transform3D &p_transform, const godot::String &p_name) override;
    const char *get_class_section() const override { return "site"; }
    void write_class_defaults(mjsDefault *p_default) override;
    void inherited_changed() override;

    void set_shape(MujocoGeom::Shape p_shape);
    MujocoGeom::Shape get_shape() const;
    void set_radius(double p_radius);
    double get_radius() const;
    void set_height(double p_height);
    double get_height() const;
    void set_size(const godot::Vector3 &p_size);
    godot::Vector3 get_size() const;

    // The site's shape in its local frame (used by the editor gizmo).
    godot::Ref<godot::Mesh> get_shape_mesh() const;

protected:
    static void _bind_methods();
    void _validate_property(godot::PropertyInfo &p_property) const;
    const MujocoAttributeTable *get_attribute_table() const override;
    godot::PackedStringArray get_property_attributes(const godot::StringName &p_property) const override;
    void overrides_changed() override;

private:
    void changed();
    void get_half_size(double r_half[3]) const;
    void write_attributes(mjsSite *p_site) const;

    // Attributes (see mujoco_element.h): shape; size0, size1, size2 (MuJoCo's size
    // parameters).
};
