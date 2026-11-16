#pragma once

#include "mujoco_element.h"

#include <godot_cpp/classes/mesh.hpp>

namespace godot {
class MeshInstance3D;
}

// A MuJoCo geom (<geom> in MJCF): a shape attached to a body, used for
// collision and to give the body its mass.
//
// The node's transform is the geom's pose relative to its body. A geom that
// isn't under a MujocoBody belongs to the world body, i.e. it is fixed scenery.
//
// Attributes (see MujocoElement for how they are stored):
//
//   shape         one of the SHAPE_ constants.
//
//   Size. These are MuJoCo's own measurements: half-sizes, not full sizes.
//   Only the ones the current shape uses are shown in the inspector.
//   radius            sphere, capsule, cylinder.
//   half_length       capsule, cylinder: half the length of the straight part,
//                     which runs along the geom's local y.
//   half_extents      box: half its size along x, y, z.
//   plane_half_size   plane: half its size along x (x) and along z (y).
//                     0 means infinite in that direction.
//   grid_spacing      plane: spacing of the grid lines MuJoCo's own viewer
//                     draws on it. No effect on the simulation.
//
//   color         the geom's colour (MJCF's rgba).
//   density       kg/m^3; the geom's mass is its volume times this.
//   mass          kg. If set, it is used instead of density. -1 = use density.
//   friction_sliding, friction_torsional, friction_rolling
//                 friction against sliding, against spinning in place, and
//                 against rolling (MJCF's three friction numbers).
//   contype, conaffinity
//                 collision filter bitmasks: two geoms collide if one's contype
//                 shares a bit with the other's conaffinity.
//
// Drawing
// -------
// A geom draws its own shape, in the editor and in the running game, in its
// rgba colour. Turn `draw_shape` off to hide that, for instance when a
// MeshInstance3D child shows a detailed model instead. (`draw_shape` is a plain
// Godot property, not a MuJoCo attribute: it has no effect on the model.)
//
// The drawing is a MeshInstance3D added as an "internal" child: it isn't saved
// with the scene and doesn't show in the scene tree.
class MujocoGeom : public MujocoElement {
    GDCLASS(MujocoGeom, MujocoElement);

public:
    // The values are MuJoCo's own (mjtGeom), so no translation is needed.
    enum Shape {
        SHAPE_PLANE = mjGEOM_PLANE,
        SHAPE_SPHERE = mjGEOM_SPHERE,
        SHAPE_CAPSULE = mjGEOM_CAPSULE,
        SHAPE_CYLINDER = mjGEOM_CYLINDER,
        SHAPE_BOX = mjGEOM_BOX,
    };

    mjsBody *add_to_spec(mjSpec *p_spec, mjsBody *p_body, const godot::Transform3D &p_transform) const override;
    mjtObj get_mujoco_type() const override { return mjOBJ_GEOM; }

    // A Godot mesh in the form of this geom's shape, at its size. Invalid for
    // a shape this class can't draw.
    godot::Ref<godot::Mesh> get_shape_mesh() const;
    // The radius of a sphere around the geom's origin that contains it.
    float get_bounding_radius() const;

    void set_draw_shape(bool p_draw_shape);
    bool get_draw_shape() const;

protected:
    static void _bind_methods();
    void _notification(int p_what);
    void attributes_changed() override;

    void _get_property_list(godot::List<godot::PropertyInfo> *r_list) const;

    const MujocoAttribute *get_attribute_table(int &r_count) const override;
    godot::Variant get_inherited_value(int p_index) const override;
    void customize_attribute_property(int p_index, godot::PropertyInfo &r_property) const override;
    bool changes_attribute_presentation(int p_index) const override;
    bool write_live(int p_index, MujocoWorld &p_world) const override;

private:
    // Whether attribute p_index is used by this geom's current shape.
    bool applies(int p_index) const;

    bool draw_shape = true;
    // The internal child that draws the shape, or nullptr while there is none.
    godot::MeshInstance3D *visual = nullptr;
    // Creates, updates or removes `visual` to match the attributes and draw_shape.
    void update_visual();

    // Reads attribute p_index out of a MuJoCo geom spec, as a Godot value.
    static godot::Variant read(const mjsGeom &p_geom, int p_index);
    // Writes a Godot value into attribute p_index of a MuJoCo geom spec.
    static void write(mjsGeom &r_geom, int p_index, const godot::Variant &p_value);
};

VARIANT_ENUM_CAST(MujocoGeom::Shape);
