#pragma once

#include "mujoco_element.h"

#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/variant/color.hpp>

// A MuJoCo geom: collision shape and mass of the body it belongs to (the
// nearest MujocoBody ancestor, or the static world body if there is none).
//
// The size is stored as MuJoCo's three size parameters (half-sizes along MuJoCo's
// local axes, which are the node's local x, -z and y), so default classes inherit
// it the way MuJoCo does. The radius, height and size properties show it in Godot
// conventions (full extents, total heights). Capsule, cylinder and plane use the
// node's local Y axis, like Godot's CapsuleMesh, CylinderMesh and PlaneMesh.
//
// A capsule's height includes its caps but is stored as the length between them,
// so a capsule that inherits its height gets longer when its radius grows (as in
// MuJoCo). A height set on the capsule itself stays as set.
class MujocoGeom : public MujocoElement {
    GDCLASS(MujocoGeom, MujocoElement);

public:
    // Values are MuJoCo's mjtGeom.
    enum Shape {
        SHAPE_PLANE = mjGEOM_PLANE,
        SHAPE_SPHERE = mjGEOM_SPHERE,
        SHAPE_CAPSULE = mjGEOM_CAPSULE,
        SHAPE_ELLIPSOID = mjGEOM_ELLIPSOID,
        SHAPE_CYLINDER = mjGEOM_CYLINDER,
        SHAPE_BOX = mjGEOM_BOX,
    };

    MujocoGeom();

    mjsBody *add_to_spec(mjSpec *p_spec, mjsBody *p_body, const godot::Transform3D &p_transform, const godot::String &p_name) override;
    const char *get_class_section() const override { return "geom"; }
    void write_class_defaults(mjsDefault *p_default) override;
    void inherited_changed() override;
    godot::PackedStringArray _get_configuration_warnings() const override;

    void set_shape(Shape p_shape);
    Shape get_shape() const;
    void set_radius(double p_radius);
    double get_radius() const;
    void set_height(double p_height);
    double get_height() const;
    void set_size(const godot::Vector3 &p_size);
    godot::Vector3 get_size() const;
    void set_density(double p_density);
    double get_density() const;
    void set_mass(double p_mass);
    double get_mass() const;
    void set_friction(const godot::Vector3 &p_friction);
    godot::Vector3 get_friction() const;
    void set_collision_layer(uint32_t p_layer);
    uint32_t get_collision_layer() const;
    void set_collision_mask(uint32_t p_mask);
    uint32_t get_collision_mask() const;
    // Contact parameters (MuJoCo's names; see its documentation for details).
    // condim: 1 frictionless, 3 sliding friction, 4 adds torsional, 6 adds rolling.
    void set_condim(int p_condim);
    int get_condim() const;
    void set_priority(int p_priority);
    int get_priority() const;
    void set_solmix(double p_value);
    double get_solmix() const;
    void set_solref_timeconst(double p_value);
    double get_solref_timeconst() const;
    void set_solref_dampratio(double p_value);
    double get_solref_dampratio() const;
    void set_solimp_dmin(double p_value);
    double get_solimp_dmin() const;
    void set_solimp_dmax(double p_value);
    double get_solimp_dmax() const;
    void set_solimp_width(double p_value);
    double get_solimp_width() const;
    void set_solimp_midpoint(double p_value);
    double get_solimp_midpoint() const;
    void set_solimp_power(double p_value);
    double get_solimp_power() const;
    void set_margin(double p_value);
    double get_margin() const;
    void set_gap(double p_value);
    double get_gap() const;
    void set_color(const godot::Color &p_color);
    godot::Color get_color() const;
    void set_show_shape(bool p_show);
    bool is_showing_shape() const;

    // The mesh drawn for this geom, in the geom's local frame (used by the editor gizmo).
    godot::Ref<godot::Mesh> get_shape_mesh() const;

    // Shared with MujocoSite, which stores its size the same way. p_half is MuJoCo's
    // size parameters.
    // A mesh of the shape, in the node's local frame.
    static godot::Ref<godot::Mesh> make_shape_mesh(Shape p_shape, const double p_half[3]);
    // The height property: capsules include their caps; for shapes without a
    // height, twice the second size parameter.
    static double height_from_half(Shape p_shape, const double p_half[3]);
    // The size property: full extents along Godot's local axes (planes: y is 0).
    static godot::Vector3 extents_from_half(Shape p_shape, const double p_half[3]);

protected:
    static void _bind_methods();
    void _validate_property(godot::PropertyInfo &p_property) const;
    const MujocoAttributeTable *get_attribute_table() const override;
    godot::PackedStringArray get_property_attributes(const godot::StringName &p_property) const override;
    void overrides_changed() override;

private:
    // A property that affects the model changed: refresh the drawn shape and recompile.
    void changed();
    void update_shape_mesh();
    // MuJoCo's size parameters, as currently resolved.
    void get_half_size(double r_half[3]) const;
    // Writes the class-able attributes (everything but the frame) into p_geom.
    void write_attributes(mjsGeom *p_geom) const;

    // Attributes (see mujoco_element.h): shape; size0, size1, size2 (MuJoCo's size
    // parameters); density (kg/m^3, used when mass is 0); mass (kg); friction
    // (slide, spin, roll); collision_layer and collision_mask (MuJoCo's contype and
    // conaffinity); color; and the contact parameters condim, priority, margin,
    // gap, solmix, solref_* and solimp_* (MuJoCo's solref and solimp, by name).
    bool show_shape = true;

    // Internal child drawing the shape; not saved with the scene.
    godot::MeshInstance3D *shape_instance = nullptr;
};

VARIANT_ENUM_CAST(MujocoGeom::Shape);
