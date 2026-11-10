#include "mujoco_geom.h"
#include "mujoco_convert.h"

#include <algorithm>

#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/box_mesh.hpp>
#include <godot_cpp/classes/capsule_mesh.hpp>
#include <godot_cpp/classes/cylinder_mesh.hpp>
#include <godot_cpp/classes/plane_mesh.hpp>
#include <godot_cpp/classes/sphere_mesh.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>

using namespace godot;

MujocoGeom::MujocoGeom()
{
    shape_instance = memnew(MeshInstance3D);
    add_child(shape_instance, false, INTERNAL_MODE_FRONT);
    update_shape_mesh();
}

mjsBody *MujocoGeom::add_to_spec(mjSpec * /*p_spec*/, mjsBody *p_body, const Transform3D &p_transform, const String &p_name)
{
    mjsGeom *geom = mjs_addGeom(p_body, nullptr);
    mjs_setName(geom->element, p_name.utf8().get_data());
    geom->type = static_cast<mjtGeom>(shape);
    mujoco_convert::to_mujoco(p_transform, geom->pos, geom->quat);

    // MuJoCo sizes are half-sizes along its local axes, which are Godot local x, -z, y.
    switch (shape) {
        case SHAPE_SPHERE:
            geom->size[0] = radius;
            break;
        case SHAPE_CAPSULE:
            geom->size[0] = radius;
            geom->size[1] = std::max(0.0, height / 2.0 - radius);
            break;
        case SHAPE_CYLINDER:
            geom->size[0] = radius;
            geom->size[1] = height / 2.0;
            break;
        case SHAPE_BOX:
        case SHAPE_ELLIPSOID:
            geom->size[0] = size.x / 2.0;
            geom->size[1] = size.z / 2.0;
            geom->size[2] = size.y / 2.0;
            break;
        case SHAPE_PLANE:
            // Collision with a plane is infinite; x/y only size its rendering in MuJoCo.
            // The third value is the rendering grid spacing.
            geom->size[0] = size.x / 2.0;
            geom->size[1] = size.z / 2.0;
            geom->size[2] = 1.0;
            break;
    }

    geom->density = density;
    if (mass > 0.0) {
        geom->mass = mass; // otherwise left unset (NaN), so MuJoCo uses density
    }
    geom->friction[0] = friction.x;
    geom->friction[1] = friction.y;
    geom->friction[2] = friction.z;
    geom->contype = static_cast<int>(collision_layer);
    geom->conaffinity = static_cast<int>(collision_mask);
    geom->rgba[0] = color.r;
    geom->rgba[1] = color.g;
    geom->rgba[2] = color.b;
    geom->rgba[3] = color.a;

    return p_body;
}

PackedStringArray MujocoGeom::_get_configuration_warnings() const
{
    PackedStringArray warnings = MujocoElement::_get_configuration_warnings();
    if (shape == SHAPE_CAPSULE && height < 2.0 * radius) {
        warnings.push_back("Capsule height is less than twice its radius; it is treated as a sphere.");
    }
    return warnings;
}

void MujocoGeom::changed()
{
    update_shape_mesh();
    update_configuration_warnings();
    request_compile();
}

// A sphere of diameter 1 stretched to p_size, baked into the vertices so that the
// mesh itself (not a node scale) has the ellipsoid's shape.
static Ref<ArrayMesh> make_ellipsoid_mesh(const Vector3 &p_size)
{
    Ref<SphereMesh> sphere;
    sphere.instantiate();
    sphere->set_radius(0.5);
    sphere->set_height(1.0);
    Array arrays = sphere->get_mesh_arrays();

    PackedVector3Array vertices = arrays[Mesh::ARRAY_VERTEX];
    PackedVector3Array normals = arrays[Mesh::ARRAY_NORMAL];
    // A zero extent would make the inverse-scale normals undefined; keep them finite.
    const Vector3 safe_size(std::max<real_t>(p_size.x, 1e-6), std::max<real_t>(p_size.y, 1e-6), std::max<real_t>(p_size.z, 1e-6));
    for (int64_t i = 0; i < vertices.size(); i++) {
        vertices.set(i, vertices[i] * p_size);
    }
    for (int64_t i = 0; i < normals.size(); i++) {
        normals.set(i, (normals[i] / safe_size).normalized());
    }
    arrays[Mesh::ARRAY_VERTEX] = vertices;
    arrays[Mesh::ARRAY_NORMAL] = normals;
    // Tangents would be wrong after stretching, and the material doesn't need them.
    arrays[Mesh::ARRAY_TANGENT] = Variant();

    Ref<ArrayMesh> mesh;
    mesh.instantiate();
    mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);
    return mesh;
}

void MujocoGeom::update_shape_mesh()
{
    Ref<Mesh> mesh;

    switch (shape) {
        case SHAPE_SPHERE: {
            Ref<SphereMesh> sphere;
            sphere.instantiate();
            sphere->set_radius(radius);
            sphere->set_height(2.0 * radius);
            mesh = sphere;
        } break;
        case SHAPE_CAPSULE: {
            Ref<CapsuleMesh> capsule;
            capsule.instantiate();
            capsule->set_radius(radius);
            capsule->set_height(std::max(height, 2.0 * radius));
            mesh = capsule;
        } break;
        case SHAPE_CYLINDER: {
            Ref<CylinderMesh> cylinder;
            cylinder.instantiate();
            cylinder->set_top_radius(radius);
            cylinder->set_bottom_radius(radius);
            cylinder->set_height(height);
            mesh = cylinder;
        } break;
        case SHAPE_BOX: {
            Ref<BoxMesh> box;
            box.instantiate();
            box->set_size(size);
            mesh = box;
        } break;
        case SHAPE_ELLIPSOID:
            mesh = make_ellipsoid_mesh(size);
            break;
        case SHAPE_PLANE: {
            Ref<PlaneMesh> plane;
            plane.instantiate();
            plane->set_size(Vector2(size.x, size.z));
            mesh = plane;
        } break;
    }

    Ref<StandardMaterial3D> material;
    material.instantiate();
    material->set_albedo(color);
    if (color.a < 1.0f) {
        material->set_transparency(BaseMaterial3D::TRANSPARENCY_ALPHA);
    }
    shape_instance->set_mesh(mesh);
    shape_instance->set_material_override(material);
    shape_instance->set_visible(show_shape);

    // The editor gizmo picks against this mesh.
    update_gizmos();
}

Ref<Mesh> MujocoGeom::get_shape_mesh() const
{
    return shape_instance->get_mesh();
}

void MujocoGeom::_validate_property(PropertyInfo &p_property) const
{
    // Only show the size properties the current shape uses.
    const bool uses_radius = shape == SHAPE_SPHERE || shape == SHAPE_CAPSULE || shape == SHAPE_CYLINDER;
    const bool uses_height = shape == SHAPE_CAPSULE || shape == SHAPE_CYLINDER;
    const bool uses_size = shape == SHAPE_BOX || shape == SHAPE_ELLIPSOID || shape == SHAPE_PLANE;

    if ((p_property.name == StringName("radius") && !uses_radius) ||
            (p_property.name == StringName("height") && !uses_height) ||
            (p_property.name == StringName("size") && !uses_size)) {
        p_property.usage = PROPERTY_USAGE_NO_EDITOR;
    }
}

void MujocoGeom::_bind_methods()
{
    BIND_ENUM_CONSTANT(SHAPE_PLANE);
    BIND_ENUM_CONSTANT(SHAPE_SPHERE);
    BIND_ENUM_CONSTANT(SHAPE_CAPSULE);
    BIND_ENUM_CONSTANT(SHAPE_ELLIPSOID);
    BIND_ENUM_CONSTANT(SHAPE_CYLINDER);
    BIND_ENUM_CONSTANT(SHAPE_BOX);

    ClassDB::bind_method(D_METHOD("set_shape", "shape"), &MujocoGeom::set_shape);
    ClassDB::bind_method(D_METHOD("get_shape"), &MujocoGeom::get_shape);
    ClassDB::bind_method(D_METHOD("set_radius", "radius"), &MujocoGeom::set_radius);
    ClassDB::bind_method(D_METHOD("get_radius"), &MujocoGeom::get_radius);
    ClassDB::bind_method(D_METHOD("set_height", "height"), &MujocoGeom::set_height);
    ClassDB::bind_method(D_METHOD("get_height"), &MujocoGeom::get_height);
    ClassDB::bind_method(D_METHOD("set_size", "size"), &MujocoGeom::set_size);
    ClassDB::bind_method(D_METHOD("get_size"), &MujocoGeom::get_size);
    ClassDB::bind_method(D_METHOD("set_density", "density"), &MujocoGeom::set_density);
    ClassDB::bind_method(D_METHOD("get_density"), &MujocoGeom::get_density);
    ClassDB::bind_method(D_METHOD("set_mass", "mass"), &MujocoGeom::set_mass);
    ClassDB::bind_method(D_METHOD("get_mass"), &MujocoGeom::get_mass);
    ClassDB::bind_method(D_METHOD("set_friction", "friction"), &MujocoGeom::set_friction);
    ClassDB::bind_method(D_METHOD("get_friction"), &MujocoGeom::get_friction);
    ClassDB::bind_method(D_METHOD("set_collision_layer", "layer"), &MujocoGeom::set_collision_layer);
    ClassDB::bind_method(D_METHOD("get_collision_layer"), &MujocoGeom::get_collision_layer);
    ClassDB::bind_method(D_METHOD("set_collision_mask", "mask"), &MujocoGeom::set_collision_mask);
    ClassDB::bind_method(D_METHOD("get_collision_mask"), &MujocoGeom::get_collision_mask);
    ClassDB::bind_method(D_METHOD("set_color", "color"), &MujocoGeom::set_color);
    ClassDB::bind_method(D_METHOD("get_color"), &MujocoGeom::get_color);
    ClassDB::bind_method(D_METHOD("set_show_shape", "show"), &MujocoGeom::set_show_shape);
    ClassDB::bind_method(D_METHOD("is_showing_shape"), &MujocoGeom::is_showing_shape);

    ADD_PROPERTY(PropertyInfo(Variant::INT, "shape", PROPERTY_HINT_ENUM, "Plane:0,Sphere:2,Capsule:3,Ellipsoid:4,Cylinder:5,Box:6"), "set_shape", "get_shape");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "radius", PROPERTY_HINT_RANGE, "0.001,10,0.001,or_greater,suffix:m"), "set_radius", "get_radius");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "height", PROPERTY_HINT_RANGE, "0.001,10,0.001,or_greater,suffix:m"), "set_height", "get_height");
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "size", PROPERTY_HINT_NONE, "suffix:m"), "set_size", "get_size");

    ADD_GROUP("Mass", "");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "density", PROPERTY_HINT_RANGE, "0,20000,0.1,or_greater,suffix:kg/m^3"), "set_density", "get_density");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "mass", PROPERTY_HINT_RANGE, "0,1000,0.001,or_greater,suffix:kg"), "set_mass", "get_mass");

    ADD_GROUP("Contact", "");
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "friction"), "set_friction", "get_friction");
    ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_layer", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_layer", "get_collision_layer");
    ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_mask", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_mask", "get_collision_mask");

    ADD_GROUP("Display", "");
    ADD_PROPERTY(PropertyInfo(Variant::COLOR, "color"), "set_color", "get_color");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "show_shape"), "set_show_shape", "is_showing_shape");
}

void MujocoGeom::set_shape(Shape p_shape)
{
    ERR_FAIL_COND_MSG(p_shape != SHAPE_PLANE && p_shape != SHAPE_SPHERE && p_shape != SHAPE_CAPSULE &&
                    p_shape != SHAPE_ELLIPSOID && p_shape != SHAPE_CYLINDER && p_shape != SHAPE_BOX,
            vformat("Unsupported geom shape: %d.", p_shape));
    shape = p_shape;
    notify_property_list_changed();
    changed();
}

MujocoGeom::Shape MujocoGeom::get_shape() const { return shape; }

void MujocoGeom::set_radius(double p_radius)
{
    ERR_FAIL_COND_MSG(p_radius <= 0.0, "Radius must be positive.");
    radius = p_radius;
    changed();
}

double MujocoGeom::get_radius() const { return radius; }

void MujocoGeom::set_height(double p_height)
{
    ERR_FAIL_COND_MSG(p_height <= 0.0, "Height must be positive.");
    height = p_height;
    changed();
}

double MujocoGeom::get_height() const { return height; }

void MujocoGeom::set_size(const Vector3 &p_size)
{
    ERR_FAIL_COND_MSG(p_size.x < 0 || p_size.y < 0 || p_size.z < 0, "Size must not be negative.");
    size = p_size;
    changed();
}

Vector3 MujocoGeom::get_size() const { return size; }

void MujocoGeom::set_density(double p_density)
{
    ERR_FAIL_COND_MSG(p_density < 0.0, "Density must not be negative.");
    density = p_density;
    changed();
}

double MujocoGeom::get_density() const { return density; }

void MujocoGeom::set_mass(double p_mass)
{
    ERR_FAIL_COND_MSG(p_mass < 0.0, "Mass must not be negative.");
    mass = p_mass;
    changed();
}

double MujocoGeom::get_mass() const { return mass; }

void MujocoGeom::set_friction(const Vector3 &p_friction)
{
    ERR_FAIL_COND_MSG(p_friction.x < 0 || p_friction.y < 0 || p_friction.z < 0, "Friction must not be negative.");
    friction = p_friction;
    changed();
}

Vector3 MujocoGeom::get_friction() const { return friction; }

void MujocoGeom::set_collision_layer(uint32_t p_layer)
{
    collision_layer = p_layer;
    changed();
}

uint32_t MujocoGeom::get_collision_layer() const { return collision_layer; }

void MujocoGeom::set_collision_mask(uint32_t p_mask)
{
    collision_mask = p_mask;
    changed();
}

uint32_t MujocoGeom::get_collision_mask() const { return collision_mask; }

void MujocoGeom::set_color(const Color &p_color)
{
    color = p_color;
    changed();
}

Color MujocoGeom::get_color() const { return color; }

void MujocoGeom::set_show_shape(bool p_show)
{
    show_shape = p_show;
    shape_instance->set_visible(show_shape); // display only; no recompile
}

bool MujocoGeom::is_showing_shape() const { return show_shape; }
