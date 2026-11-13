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

using namespace godot;

namespace {

// Indices into ATTRIBUTES below. The two must stay in the same order.
enum Attribute {
    SHAPE,
    SIZE,
    RGBA,
    DENSITY,
    MASS,
    FRICTION,
    CONTYPE,
    CONAFFINITY,
    ATTRIBUTE_COUNT,
};

const MujocoAttribute ATTRIBUTES[] = {
    { "shape", Variant::INT, PROPERTY_HINT_ENUM, "Plane:0,Sphere:2,Capsule:3,Cylinder:5,Box:6" },
    { "size", Variant::VECTOR3, PROPERTY_HINT_NONE, "" },
    { "rgba", Variant::COLOR, PROPERTY_HINT_NONE, "" },
    { "density", Variant::FLOAT, PROPERTY_HINT_NONE, "" },
    { "mass", Variant::FLOAT, PROPERTY_HINT_NONE, "" },
    { "friction", Variant::VECTOR3, PROPERTY_HINT_NONE, "" },
    { "contype", Variant::INT, PROPERTY_HINT_LAYERS_3D_PHYSICS, "" },
    { "conaffinity", Variant::INT, PROPERTY_HINT_LAYERS_3D_PHYSICS, "" },
};

static_assert(sizeof(ATTRIBUTES) / sizeof(ATTRIBUTES[0]) == ATTRIBUTE_COUNT, "ATTRIBUTES and enum Attribute must match");

// The enum hint above spells out MuJoCo's values.
static_assert(mjGEOM_PLANE == 0 && mjGEOM_SPHERE == 2 && mjGEOM_CAPSULE == 3 && mjGEOM_CYLINDER == 5 && mjGEOM_BOX == 6);

// What a geom's size is when nothing sets it. MuJoCo's own default is 0, which
// it refuses to compile, so a new geom would be an error until it was sized.
const Vector3 DEFAULT_SIZE(0.5, 0.5, 0.5);

// Sizes of these shapes are extents along the three axes, so they have to be
// reordered between Godot's axes and MuJoCo's. (The others are a radius and a
// half-length, which don't depend on the axis convention.)
bool has_axis_extents(mjtGeom p_type) {
    return p_type == mjGEOM_BOX || p_type == mjGEOM_ELLIPSOID || p_type == mjGEOM_PLANE;
}

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

Variant MujocoGeom::read(const mjsGeom &p_geom, int p_index)
{
    switch (p_index) {
        case SHAPE:
            return int(p_geom.type);
        case SIZE:
            // MuJoCo's y is Godot's depth (z) and MuJoCo's z is Godot's up (y).
            // An extent has no direction, so no sign change, just the swap.
            if (has_axis_extents(p_geom.type)) {
                return Vector3(p_geom.size[0], p_geom.size[2], p_geom.size[1]);
            }
            return Vector3(p_geom.size[0], p_geom.size[1], p_geom.size[2]);
        case RGBA:
            return Color(p_geom.rgba[0], p_geom.rgba[1], p_geom.rgba[2], p_geom.rgba[3]);
        case DENSITY:
            return p_geom.density;
        case MASS:
            // MuJoCo marks "no mass given, use density" with NaN. Show that as -1.
            return std::isnan(p_geom.mass) ? -1.0 : p_geom.mass;
        case FRICTION:
            return Vector3(p_geom.friction[0], p_geom.friction[1], p_geom.friction[2]);
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
        case SIZE: {
            const Vector3 size = p_value;
            r_geom.size[0] = size.x;
            if (has_axis_extents(r_geom.type)) {
                r_geom.size[1] = size.z;
                r_geom.size[2] = size.y;
            } else {
                r_geom.size[1] = size.y;
                r_geom.size[2] = size.z;
            }
        } break;
        case RGBA: {
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
        case FRICTION: {
            const Vector3 friction = p_value;
            r_geom.friction[0] = friction.x;
            r_geom.friction[1] = friction.y;
            r_geom.friction[2] = friction.z;
        } break;
        case CONTYPE:
            r_geom.contype = p_value;
            break;
        case CONAFFINITY:
            r_geom.conaffinity = p_value;
            break;
    }
}

// An attribute that isn't overridden has MuJoCo's built-in default, read from a
// geom spec that MuJoCo itself fills in. Size is the one exception.
Variant MujocoGeom::get_inherited_value(int p_index) const
{
    if (p_index == SIZE) {
        return DEFAULT_SIZE;
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

    // Then only the overridden attributes are written. SHAPE comes before SIZE
    // in the table, so the shape is known by the time the size is written.
    for (int i = 0; i < ATTRIBUTE_COUNT; i++) {
        Variant value;
        if (get_override(i, value)) {
            write(*geom, i, value);
        } else if (i == SIZE) {
            // Our default size differs from MuJoCo's, so it has to be written too.
            write(*geom, SIZE, DEFAULT_SIZE);
        }
    }

    // A geom can't contain other elements.
    return nullptr;
}

// ---- Drawing ----

namespace {

// How large an infinite plane (half-extent 0) is drawn, in metres across.
const float INFINITE_PLANE_DRAWN_SIZE = 200.0f;

// A Godot mesh in the form of the given shape. Godot's primitive meshes take
// full sizes where MuJoCo's sizes are half-sizes, hence the doubling. They also
// already point the right way: capsules and cylinders run along y, and a plane
// faces +y.
Ref<Mesh> make_mesh(int p_shape, const Vector3 &p_size)
{
    switch (p_shape) {
        case MujocoGeom::SHAPE_SPHERE: {
            Ref<SphereMesh> mesh;
            mesh.instantiate();
            mesh->set_radius(p_size.x);
            mesh->set_height(2 * p_size.x);
            return mesh;
        }
        case MujocoGeom::SHAPE_CAPSULE: {
            Ref<CapsuleMesh> mesh;
            mesh.instantiate();
            mesh->set_radius(p_size.x);
            // MuJoCo's half-length is of the cylindrical part only; Godot's
            // height includes the two rounded caps.
            mesh->set_height(2 * p_size.y + 2 * p_size.x);
            return mesh;
        }
        case MujocoGeom::SHAPE_CYLINDER: {
            Ref<CylinderMesh> mesh;
            mesh.instantiate();
            mesh->set_top_radius(p_size.x);
            mesh->set_bottom_radius(p_size.x);
            mesh->set_height(2 * p_size.y);
            return mesh;
        }
        case MujocoGeom::SHAPE_BOX: {
            Ref<BoxMesh> mesh;
            mesh.instantiate();
            mesh->set_size(2 * p_size);
            return mesh;
        }
        case MujocoGeom::SHAPE_PLANE: {
            Ref<PlaneMesh> mesh;
            mesh.instantiate();
            const float width = p_size.x > 0 ? 2 * p_size.x : INFINITE_PLANE_DRAWN_SIZE;
            const float depth = p_size.z > 0 ? 2 * p_size.z : INFINITE_PLANE_DRAWN_SIZE;
            mesh->set_size(Vector2(width, depth));
            return mesh;
        }
    }
    // A shape this class doesn't draw.
    return Ref<Mesh>();
}

} // namespace

Ref<Mesh> MujocoGeom::get_shape_mesh() const
{
    return make_mesh(get_attribute(SHAPE), get_attribute(SIZE));
}

void MujocoGeom::update_visual()
{
    // In the editor, also rebuild what a click on this geom is tested against
    // (see MujocoGeomGizmoPlugin). Does nothing in a running game.
    update_gizmos();

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

    const Color color = get_attribute(RGBA);
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
