#pragma once

#include "mujoco_element.h"

#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/variant/color.hpp>

// A MuJoCo geom: collision shape and mass of the body it belongs to (the
// nearest MujocoBody ancestor, or the static world body if there is none).
//
// Sizes use Godot conventions (full extents, total heights); the conversion to
// MuJoCo's half-sizes happens in add_to_spec(). Capsule, cylinder and plane use
// the node's local Y axis, like Godot's CapsuleMesh, CylinderMesh and PlaneMesh.
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
    void set_color(const godot::Color &p_color);
    godot::Color get_color() const;
    void set_show_shape(bool p_show);
    bool is_showing_shape() const;

    // The mesh drawn for this geom, in the geom's local frame (used by the editor gizmo).
    godot::Ref<godot::Mesh> get_shape_mesh() const;

    // Shared with MujocoSite, which uses the same shapes and size conventions.
    // MuJoCo's size parameters (half-sizes along MuJoCo's local axes) for a shape.
    static void shape_size_to_mujoco(Shape p_shape, double p_radius, double p_height, const godot::Vector3 &p_size, double r_size[3]);
    // A mesh of the shape, in the node's local frame.
    static godot::Ref<godot::Mesh> make_shape_mesh(Shape p_shape, double p_radius, double p_height, const godot::Vector3 &p_size);

protected:
    static void _bind_methods();
    void _validate_property(godot::PropertyInfo &p_property) const;

private:
    // A property that affects the model changed: refresh the drawn shape and recompile.
    void changed();
    void update_shape_mesh();

    Shape shape = SHAPE_BOX;
    double radius = 0.5;                  // sphere, capsule, cylinder
    double height = 2.0;                  // capsule (total, including caps), cylinder
    godot::Vector3 size{ 1, 1, 1 };       // box and ellipsoid: full extents; plane: x and z extents
    double density = 1000.0;              // kg/m^3, used when mass is 0
    double mass = 0.0;                    // kg; 0 means compute from density
    godot::Vector3 friction{ 1, 0.005, 0.0001 }; // slide, spin, roll (MuJoCo defaults)
    uint32_t collision_layer = 1;         // MuJoCo contype
    uint32_t collision_mask = 1;          // MuJoCo conaffinity
    godot::Color color{ 0.5, 0.5, 0.5, 1.0 };
    bool show_shape = true;

    // Internal child drawing the shape; not saved with the scene.
    godot::MeshInstance3D *shape_instance = nullptr;
};

VARIANT_ENUM_CAST(MujocoGeom::Shape);
