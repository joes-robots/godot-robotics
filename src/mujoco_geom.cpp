#include "mujoco_geom.h"

#include <cmath>

#include <godot_cpp/classes/box_mesh.hpp>
#include <godot_cpp/classes/capsule_mesh.hpp>
#include <godot_cpp/classes/cylinder_mesh.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/plane_mesh.hpp>
#include <godot_cpp/classes/sphere_mesh.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>

#include "mujoco_convert.h"
#include "mujoco_joint.h"
#include "mujoco_world.h"

using namespace godot;

namespace {

// Indices into ATTRIBUTES below. The two must stay in the same order.
enum Attribute {
    SHAPE,
    // Size. MuJoCo has one "size" of three numbers whose meaning depends on the
    // shape. Here each meaning is its own attribute, and only the ones the
    // current shape uses are shown.
    RADIUS,
    HALF_LENGTH,
    HALF_EXTENTS,
    PLANE_HALF_SIZE,
    GRID_SPACING,
    COLOR,
    DENSITY,
    MASS,
    FRICTION_SLIDING,
    FRICTION_TORSIONAL,
    FRICTION_ROLLING,
    CONTYPE,
    CONAFFINITY,
    ATTRIBUTE_COUNT,
};

const MujocoAttribute ATTRIBUTES[] = {
    { "shape", Variant::INT, PROPERTY_HINT_ENUM, "Plane:0,Sphere:2,Capsule:3,Cylinder:5,Box:6" },
    { "radius", Variant::FLOAT, PROPERTY_HINT_NONE, "" },
    { "half_length", Variant::FLOAT, PROPERTY_HINT_NONE, "" },
    { "half_extents", Variant::VECTOR3, PROPERTY_HINT_NONE, "" },
    { "plane_half_size", Variant::VECTOR2, PROPERTY_HINT_NONE, "" },
    { "grid_spacing", Variant::FLOAT, PROPERTY_HINT_NONE, "" },
    { "color", Variant::COLOR, PROPERTY_HINT_NONE, "" },
    { "density", Variant::FLOAT, PROPERTY_HINT_NONE, "" },
    { "mass", Variant::FLOAT, PROPERTY_HINT_NONE, "" },
    { "friction_sliding", Variant::FLOAT, PROPERTY_HINT_NONE, "" },
    { "friction_torsional", Variant::FLOAT, PROPERTY_HINT_NONE, "" },
    { "friction_rolling", Variant::FLOAT, PROPERTY_HINT_NONE, "" },
    { "contype", Variant::INT, PROPERTY_HINT_LAYERS_3D_PHYSICS, "" },
    { "conaffinity", Variant::INT, PROPERTY_HINT_LAYERS_3D_PHYSICS, "" },
};

static_assert(sizeof(ATTRIBUTES) / sizeof(ATTRIBUTES[0]) == ATTRIBUTE_COUNT, "ATTRIBUTES and enum Attribute must match");

// The enum hint above spells out MuJoCo's values.
static_assert(mjGEOM_PLANE == 0 && mjGEOM_SPHERE == 2 && mjGEOM_CAPSULE == 3 && mjGEOM_CYLINDER == 5 && mjGEOM_BOX == 6);

bool is_size_attribute(int p_index) {
    return p_index >= RADIUS && p_index <= GRID_SPACING;
}

// How large an infinite plane (half-size 0) is drawn, in metres across.
const float INFINITE_PLANE_DRAWN_SIZE = 200.0f;

} // namespace

void MujocoGeom::_bind_methods()
{
    BIND_ENUM_CONSTANT(SHAPE_PLANE);
    BIND_ENUM_CONSTANT(SHAPE_SPHERE);
    BIND_ENUM_CONSTANT(SHAPE_CAPSULE);
    BIND_ENUM_CONSTANT(SHAPE_CYLINDER);
    BIND_ENUM_CONSTANT(SHAPE_BOX);

    ClassDB::bind_method(D_METHOD("set_draw_shape", "draw_shape"), &MujocoGeom::set_draw_shape);
    ClassDB::bind_method(D_METHOD("get_draw_shape"), &MujocoGeom::get_draw_shape);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "draw_shape"), "set_draw_shape", "get_draw_shape");
}

// Lists the attributes here, so the inspector shows them under "MujocoGeom".
void MujocoGeom::_get_property_list(List<PropertyInfo> *r_list) const
{
    get_attribute_property_list(r_list);
}

const MujocoAttribute *MujocoGeom::get_attribute_table(int &r_count) const
{
    r_count = ATTRIBUTE_COUNT;
    return ATTRIBUTES;
}

// Which size attributes each shape uses:
//
//                     sphere  capsule  cylinder  box  plane
//   radius             yes     yes      yes       -    -
//   half_length        -       yes      yes       -    -
//   half_extents       -       -        -         yes  -
//   plane_half_size    -       -        -         -    yes
//   grid_spacing       -       -        -         -    yes
//
// Every other attribute applies to every shape.
bool MujocoGeom::applies(int p_index) const
{
    const int shape = get_attribute(SHAPE);
    switch (p_index) {
        case RADIUS:
            return shape == SHAPE_SPHERE || shape == SHAPE_CAPSULE || shape == SHAPE_CYLINDER;
        case HALF_LENGTH:
            return shape == SHAPE_CAPSULE || shape == SHAPE_CYLINDER;
        case HALF_EXTENTS:
            return shape == SHAPE_BOX;
        case PLANE_HALF_SIZE:
        case GRID_SPACING:
            return shape == SHAPE_PLANE;
    }
    return true;
}

// Size attributes of other shapes are hidden, not greyed out: they are
// alternatives to one another, and listing them all would bury the ones in use.
void MujocoGeom::customize_attribute_property(int p_index, PropertyInfo &r_property) const
{
    if (!applies(p_index)) {
        r_property.usage = PROPERTY_USAGE_NONE;
    }
}

// Changing the shape changes which size attributes are shown.
bool MujocoGeom::changes_attribute_presentation(int p_index) const
{
    return p_index == SHAPE;
}

// Where each size attribute sits in MuJoCo's size[3]:
//
//   sphere             size = (radius)
//   capsule, cylinder  size = (radius, half_length)
//   box                size = half-extents along MuJoCo's x, y, z
//   plane              size = (half-size x, half-size y, grid spacing)
//
// MuJoCo's y is Godot's depth (z) and MuJoCo's z is Godot's up (y), so a box's
// half_extents (given along Godot's axes) has its y and z swapped. An extent
// has no direction, so there is no sign change.
Variant MujocoGeom::read(const mjsGeom &p_geom, int p_index)
{
    switch (p_index) {
        case SHAPE:
            return int(p_geom.type);
        case RADIUS:
            return p_geom.size[0];
        case HALF_LENGTH:
            return p_geom.size[1];
        case HALF_EXTENTS:
            return Vector3(p_geom.size[0], p_geom.size[2], p_geom.size[1]);
        case PLANE_HALF_SIZE:
            return Vector2(p_geom.size[0], p_geom.size[1]);
        case GRID_SPACING:
            return p_geom.size[2];
        case COLOR:
            return Color(p_geom.rgba[0], p_geom.rgba[1], p_geom.rgba[2], p_geom.rgba[3]);
        case DENSITY:
            return p_geom.density;
        case MASS:
            // MuJoCo marks "no mass given, use density" with NaN. Show that as -1.
            return std::isnan(p_geom.mass) ? -1.0 : p_geom.mass;
        case FRICTION_SLIDING:
            return p_geom.friction[0];
        case FRICTION_TORSIONAL:
            return p_geom.friction[1];
        case FRICTION_ROLLING:
            return p_geom.friction[2];
        case CONTYPE:
            return p_geom.contype;
        case CONAFFINITY:
            return p_geom.conaffinity;
    }
    return Variant();
}

void MujocoGeom::write(mjsGeom &r_geom, int p_index, const Variant &p_value)
{
    switch (p_index) {
        case SHAPE:
            r_geom.type = mjtGeom(int(p_value));
            break;
        case RADIUS:
            r_geom.size[0] = p_value;
            break;
        case HALF_LENGTH:
            r_geom.size[1] = p_value;
            break;
        case HALF_EXTENTS: {
            const Vector3 half_extents = p_value;
            r_geom.size[0] = half_extents.x;
            r_geom.size[1] = half_extents.z;
            r_geom.size[2] = half_extents.y;
        } break;
        case PLANE_HALF_SIZE: {
            const Vector2 half_size = p_value;
            r_geom.size[0] = half_size.x;
            r_geom.size[1] = half_size.y;
        } break;
        case GRID_SPACING:
            r_geom.size[2] = p_value;
            break;
        case COLOR: {
            const Color color = p_value;
            r_geom.rgba[0] = color.r;
            r_geom.rgba[1] = color.g;
            r_geom.rgba[2] = color.b;
            r_geom.rgba[3] = color.a;
        } break;
        case DENSITY:
            r_geom.density = p_value;
            break;
        case MASS: {
            const double mass = p_value;
            r_geom.mass = mass < 0 ? std::nan("") : mass;
        } break;
        case FRICTION_SLIDING:
            r_geom.friction[0] = p_value;
            break;
        case FRICTION_TORSIONAL:
            r_geom.friction[1] = p_value;
            break;
        case FRICTION_ROLLING:
            r_geom.friction[2] = p_value;
            break;
        case CONTYPE:
            r_geom.contype = p_value;
            break;
        case CONAFFINITY:
            r_geom.conaffinity = p_value;
            break;
    }
}

// An attribute that isn't overridden has MuJoCo's built-in default, read from a
// geom spec that MuJoCo itself fills in. The size attributes are the exception:
// MuJoCo's default size is 0, which it refuses to compile, so a new geom would
// be an error until it was sized. These defaults make a new geom of any shape
// valid as it is.
Variant MujocoGeom::get_inherited_value(int p_index) const
{
    switch (p_index) {
        case RADIUS:
            return 0.5;
        case HALF_LENGTH:
            return 0.5;
        case HALF_EXTENTS:
            return Vector3(0.5, 0.5, 0.5);
        case PLANE_HALF_SIZE:
            return Vector2(0, 0); // infinite
        case GRID_SPACING:
            return 1.0;
    }
    mjsGeom defaults;
    mjs_defaultGeom(&defaults);
    return read(defaults, p_index);
}

mjsBody *MujocoGeom::add_to_spec(mjSpec *p_spec, mjsBody *p_body, const Transform3D &p_transform) const
{
    // The new geom starts with MuJoCo's defaults for everything.
    mjsGeom *geom = mjs_addGeom(p_body, nullptr);
    mjs_setName(geom->element, get_mujoco_name().utf8().get_data());
    mujoco_convert::to_mujoco(p_transform, geom->pos, geom->quat);

    for (int i = 0; i < ATTRIBUTE_COUNT; i++) {
        if (!applies(i)) {
            continue; // a size attribute of some other shape
        }
        if (is_size_attribute(i)) {
            // Our size defaults differ from MuJoCo's, so the size in effect is
            // always written, overridden or not.
            write(*geom, i, get_attribute(i));
            continue;
        }
        // For everything else only overrides are written.
        Variant value;
        if (get_override(i, value)) {
            write(*geom, i, value);
        }
    }

    // A geom can't contain other elements.
    return nullptr;
}

// ---- Drawing ----

// A Godot mesh in the form of this geom. Godot's primitive meshes take full
// sizes where MuJoCo's sizes are half-sizes, hence the doubling. They also
// already point the right way: capsules and cylinders run along y, and a plane
// faces +y.
Ref<Mesh> MujocoGeom::get_shape_mesh() const
{
    const float radius = get_attribute(RADIUS);
    const float half_length = get_attribute(HALF_LENGTH);

    switch (int(get_attribute(SHAPE))) {
        case SHAPE_SPHERE: {
            Ref<SphereMesh> mesh;
            mesh.instantiate();
            mesh->set_radius(radius);
            mesh->set_height(2 * radius);
            return mesh;
        }
        case SHAPE_CAPSULE: {
            Ref<CapsuleMesh> mesh;
            mesh.instantiate();
            mesh->set_radius(radius);
            // MuJoCo's half-length is of the cylindrical part only; Godot's
            // height includes the two rounded caps.
            mesh->set_height(2 * half_length + 2 * radius);
            return mesh;
        }
        case SHAPE_CYLINDER: {
            Ref<CylinderMesh> mesh;
            mesh.instantiate();
            mesh->set_top_radius(radius);
            mesh->set_bottom_radius(radius);
            mesh->set_height(2 * half_length);
            return mesh;
        }
        case SHAPE_BOX: {
            Ref<BoxMesh> mesh;
            mesh.instantiate();
            mesh->set_size(2 * Vector3(get_attribute(HALF_EXTENTS)));
            return mesh;
        }
        case SHAPE_PLANE: {
            Ref<PlaneMesh> mesh;
            mesh.instantiate();
            const Vector2 half_size = get_attribute(PLANE_HALF_SIZE);
            const float width = half_size.x > 0 ? 2 * half_size.x : INFINITE_PLANE_DRAWN_SIZE;
            const float depth = half_size.y > 0 ? 2 * half_size.y : INFINITE_PLANE_DRAWN_SIZE;
            mesh->set_size(Vector2(width, depth));
            return mesh;
        }
    }
    // A shape this class doesn't draw.
    return Ref<Mesh>();
}

// The radius of a sphere around the geom's own origin that contains it.
// Planes, which may be infinite, report 0.
float MujocoGeom::get_bounding_radius() const
{
    const float radius = get_attribute(RADIUS);
    const float half_length = get_attribute(HALF_LENGTH);
    switch (int(get_attribute(SHAPE))) {
        case SHAPE_SPHERE:
            return radius;
        case SHAPE_CAPSULE:
            return half_length + radius;
        case SHAPE_CYLINDER:
            return Vector2(radius, half_length).length();
        case SHAPE_BOX:
            return Vector3(get_attribute(HALF_EXTENTS)).length();
    }
    return 0.0f;
}

void MujocoGeom::update_visual()
{
    // In the editor, also rebuild what a click on this geom is tested against
    // (see MujocoGeomGizmoPlugin). Does nothing in a running game.
    update_gizmos();
    // Joints on the same body size their gizmos by this geom.
    if (const Node *parent = get_parent()) {
        for (int i = 0; i < parent->get_child_count(); i++) {
            if (MujocoJoint *joint = Object::cast_to<MujocoJoint>(parent->get_child(i))) {
                joint->update_gizmos();
            }
        }
    }

    if (!draw_shape) {
        if (visual) {
            visual->queue_free();
            visual = nullptr;
        }
        return;
    }

    if (!visual) {
        // Created the first time the geom is in the tree, then kept.
        if (!is_inside_tree()) {
            return;
        }
        visual = memnew(MeshInstance3D);
        // An internal child is left out of saved scenes, the scene tree dock,
        // and get_children(), so it stays this class's private business.
        add_child(visual, false, INTERNAL_MODE_FRONT);
    }

    visual->set_mesh(get_shape_mesh());

    const Color color = get_attribute(COLOR);
    Ref<StandardMaterial3D> material;
    material.instantiate();
    material->set_albedo(color);
    if (color.a < 1.0f) {
        material->set_transparency(BaseMaterial3D::TRANSPARENCY_ALPHA);
    }
    visual->set_material_override(material);
}

void MujocoGeom::attributes_changed()
{
    update_visual();
}

void MujocoGeom::_notification(int p_what)
{
    if (p_what == NOTIFICATION_ENTER_TREE) {
        update_visual();
    }
}

void MujocoGeom::set_draw_shape(bool p_draw_shape)
{
    draw_shape = p_draw_shape;
    update_visual();
}

bool MujocoGeom::get_draw_shape() const
{
    return draw_shape;
}

// ---- Live attributes ----

// Friction and colour are plain numbers in the compiled model. Shape, size,
// density and mass are not: they determine the body's inertia and what the
// geom collides with, which are worked out when the model is compiled.
bool MujocoGeom::write_live(int p_index, MujocoWorld &p_world) const
{
    mjModel *model = p_world.get_model();
    const int id = get_mujoco_id();

    switch (p_index) {
        case FRICTION_SLIDING:
            model->geom_friction[3 * id] = get_attribute(FRICTION_SLIDING);
            return true;
        case FRICTION_TORSIONAL:
            model->geom_friction[3 * id + 1] = get_attribute(FRICTION_TORSIONAL);
            return true;
        case FRICTION_ROLLING:
            model->geom_friction[3 * id + 2] = get_attribute(FRICTION_ROLLING);
            return true;
        case COLOR: {
            // The drawn shape is updated separately (attributes_changed); this
            // keeps the model's copy in step.
            const Color color = get_attribute(COLOR);
            model->geom_rgba[4 * id] = color.r;
            model->geom_rgba[4 * id + 1] = color.g;
            model->geom_rgba[4 * id + 2] = color.b;
            model->geom_rgba[4 * id + 3] = color.a;
            return true;
        }
    }
    return false;
}
