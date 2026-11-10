#include "mujoco_geom.h"
#include "mujoco_convert.h"

#include <algorithm>
#include <limits>

#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/box_mesh.hpp>
#include <godot_cpp/classes/capsule_mesh.hpp>
#include <godot_cpp/classes/cylinder_mesh.hpp>
#include <godot_cpp/classes/plane_mesh.hpp>
#include <godot_cpp/classes/sphere_mesh.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>

using namespace godot;

namespace {

// The base sizes (0.5 m half-sizes) make a 1 m box, a 0.5 m sphere, a 2 m capsule
// and a 1 m cylinder.
const MujocoAttribute GEOM_ATTRIBUTES[] = {
    { "shape", Variant::INT, { MujocoGeom::SHAPE_BOX } },
    { "size0", Variant::FLOAT, { 0.5 } },
    { "size1", Variant::FLOAT, { 0.5 } },
    { "size2", Variant::FLOAT, { 0.5 } },
    { "density", Variant::FLOAT, { 1000.0 } },
    { "mass", Variant::FLOAT, { 0.0 } },
    { "friction", Variant::VECTOR3, { 1, 0.005, 0.0001 } }, // MuJoCo's defaults
    { "collision_layer", Variant::INT, { 1 } },
    { "collision_mask", Variant::INT, { 1 } },
    { "color", Variant::COLOR, { 0.5, 0.5, 0.5, 1.0 } },
};
const MujocoAttributeTable GEOM_TABLE = MUJOCO_ATTRIBUTE_TABLE(GEOM_ATTRIBUTES);

} // namespace

const MujocoAttributeTable *MujocoGeom::get_attribute_table() const
{
    return &GEOM_TABLE;
}

PackedStringArray MujocoGeom::get_property_attributes(const StringName &p_property) const
{
    PackedStringArray keys;
    if (p_property == StringName("radius")) {
        keys.push_back("size0");
    } else if (p_property == StringName("height")) {
        keys.push_back("size1");
    } else if (p_property == StringName("size")) {
        keys.push_back("size0");
        keys.push_back("size1");
        if (get_shape() != SHAPE_PLANE) {
            keys.push_back("size2");
        }
    } else {
        keys = MujocoElement::get_property_attributes(p_property);
    }
    return keys;
}

void MujocoGeom::overrides_changed()
{
    notify_property_list_changed();
    changed();
}

void MujocoGeom::get_half_size(double r_half[3]) const
{
    r_half[0] = get_attribute("size0");
    r_half[1] = get_attribute("size1");
    r_half[2] = get_attribute("size2");
}

double MujocoGeom::height_from_half(Shape p_shape, const double p_half[3])
{
    return p_shape == SHAPE_CAPSULE ? 2.0 * (p_half[1] + p_half[0]) : 2.0 * p_half[1];
}

Vector3 MujocoGeom::extents_from_half(Shape p_shape, const double p_half[3])
{
    // MuJoCo's local x, y, z are Godot's local x, -z, y.
    return Vector3(2.0 * p_half[0], p_shape == SHAPE_PLANE ? 0.0 : 2.0 * p_half[2], 2.0 * p_half[1]);
}

MujocoGeom::MujocoGeom()
{
    shape_instance = memnew(MeshInstance3D);
    add_child(shape_instance, false, INTERNAL_MODE_FRONT);
    update_shape_mesh();
}

mjsBody *MujocoGeom::add_to_spec(mjSpec *p_spec, mjsBody *p_body, const Transform3D &p_transform, const String &p_name)
{
    mjsGeom *geom = mjs_addGeom(p_body, get_spec_default(p_spec));
    mjs_setName(geom->element, p_name.utf8().get_data());
    write_attributes(geom);
    mujoco_convert::to_mujoco(p_transform, geom->pos, geom->quat);
    return p_body;
}

void MujocoGeom::write_attributes(mjsGeom *p_geom) const
{
    p_geom->type = static_cast<mjtGeom>(get_shape());
    get_half_size(p_geom->size);
    p_geom->density = get_density();
    // Mass 0 means "from density", which MuJoCo marks with NaN.
    const double mass = get_mass();
    p_geom->mass = mass > 0.0 ? mass : std::numeric_limits<double>::quiet_NaN();
    const Vector3 friction = get_friction();
    p_geom->friction[0] = friction.x;
    p_geom->friction[1] = friction.y;
    p_geom->friction[2] = friction.z;
    p_geom->contype = static_cast<int>(get_collision_layer());
    p_geom->conaffinity = static_cast<int>(get_collision_mask());
    const Color color = get_color();
    p_geom->rgba[0] = color.r;
    p_geom->rgba[1] = color.g;
    p_geom->rgba[2] = color.b;
    p_geom->rgba[3] = color.a;
}

void MujocoGeom::write_class_defaults(mjsDefault *p_default)
{
    write_attributes(p_default->geom);
}

void MujocoGeom::inherited_changed()
{
    MujocoElement::inherited_changed();
    // The drawn shape may be inherited.
    update_shape_mesh();
}

PackedStringArray MujocoGeom::_get_configuration_warnings() const
{
    return MujocoElement::_get_configuration_warnings();
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

Ref<Mesh> MujocoGeom::make_shape_mesh(Shape p_shape, const double p_half[3])
{
    const double radius = p_half[0];
    const double height = height_from_half(p_shape, p_half);
    const Vector3 size = extents_from_half(p_shape, p_half);
    Ref<Mesh> mesh;

    switch (p_shape) {
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
    return mesh;
}

void MujocoGeom::update_shape_mesh()
{
    double half[3];
    get_half_size(half);
    const Ref<Mesh> mesh = make_shape_mesh(get_shape(), half);
    const Color color = get_color();

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
    const Shape shape = get_shape();
    const bool uses_radius = shape == SHAPE_SPHERE || shape == SHAPE_CAPSULE || shape == SHAPE_CYLINDER;
    const bool uses_height = shape == SHAPE_CAPSULE || shape == SHAPE_CYLINDER;
    const bool uses_size = shape == SHAPE_BOX || shape == SHAPE_ELLIPSOID || shape == SHAPE_PLANE;

    if ((p_property.name == StringName("radius") && !uses_radius) ||
            (p_property.name == StringName("height") && !uses_height) ||
            (p_property.name == StringName("size") && !uses_size)) {
        p_property.usage = PROPERTY_USAGE_NONE; // hidden; the value is stored in overrides
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

    // Attribute properties are stored in "overrides", so they are editor-only here.
    ADD_PROPERTY(PropertyInfo(Variant::INT, "shape", PROPERTY_HINT_ENUM, "Plane:0,Sphere:2,Capsule:3,Ellipsoid:4,Cylinder:5,Box:6", PROPERTY_USAGE_EDITOR), "set_shape", "get_shape");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "radius", PROPERTY_HINT_RANGE, "0.001,10,0.001,or_greater,suffix:m", PROPERTY_USAGE_EDITOR), "set_radius", "get_radius");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "height", PROPERTY_HINT_RANGE, "0.001,10,0.001,or_greater,suffix:m", PROPERTY_USAGE_EDITOR), "set_height", "get_height");
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "size", PROPERTY_HINT_NONE, "suffix:m", PROPERTY_USAGE_EDITOR), "set_size", "get_size");

    ADD_GROUP("Mass", "");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "density", PROPERTY_HINT_RANGE, "0,20000,0.1,or_greater,suffix:kg/m^3", PROPERTY_USAGE_EDITOR), "set_density", "get_density");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "mass", PROPERTY_HINT_RANGE, "0,1000,0.001,or_greater,suffix:kg", PROPERTY_USAGE_EDITOR), "set_mass", "get_mass");

    ADD_GROUP("Contact", "");
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "friction", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR), "set_friction", "get_friction");
    ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_layer", PROPERTY_HINT_LAYERS_3D_PHYSICS, "", PROPERTY_USAGE_EDITOR), "set_collision_layer", "get_collision_layer");
    ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_mask", PROPERTY_HINT_LAYERS_3D_PHYSICS, "", PROPERTY_USAGE_EDITOR), "set_collision_mask", "get_collision_mask");

    ADD_GROUP("Display", "");
    ADD_PROPERTY(PropertyInfo(Variant::COLOR, "color", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR), "set_color", "get_color");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "show_shape"), "set_show_shape", "is_showing_shape");
}

void MujocoGeom::set_shape(Shape p_shape)
{
    ERR_FAIL_COND_MSG(p_shape != SHAPE_PLANE && p_shape != SHAPE_SPHERE && p_shape != SHAPE_CAPSULE &&
                    p_shape != SHAPE_ELLIPSOID && p_shape != SHAPE_CYLINDER && p_shape != SHAPE_BOX,
            vformat("Unsupported geom shape: %d.", p_shape));
    set_attribute("shape", p_shape);
    notify_property_list_changed();
    changed();
}

MujocoGeom::Shape MujocoGeom::get_shape() const { return static_cast<Shape>(static_cast<int64_t>(get_attribute("shape"))); }

void MujocoGeom::set_radius(double p_radius)
{
    ERR_FAIL_COND_MSG(p_radius <= 0.0, "Radius must be positive.");
    // A capsule keeps a height set on it (see the class comment).
    const bool keep_height = get_shape() == SHAPE_CAPSULE && has_override("size1");
    const double height = get_height();
    set_attribute("size0", p_radius);
    if (keep_height) {
        set_attribute("size1", std::max(0.0, height / 2.0 - p_radius));
    }
    changed();
}

double MujocoGeom::get_radius() const { return get_attribute("size0"); }

void MujocoGeom::set_height(double p_height)
{
    ERR_FAIL_COND_MSG(p_height <= 0.0, "Height must be positive.");
    const double half = get_shape() == SHAPE_CAPSULE ? std::max(0.0, p_height / 2.0 - get_radius()) : p_height / 2.0;
    set_attribute("size1", half);
    changed();
}

double MujocoGeom::get_height() const
{
    double half[3];
    get_half_size(half);
    return height_from_half(get_shape(), half);
}

void MujocoGeom::set_size(const Vector3 &p_size)
{
    ERR_FAIL_COND_MSG(p_size.x < 0 || p_size.y < 0 || p_size.z < 0, "Size must not be negative.");
    set_attribute("size0", p_size.x / 2.0);
    set_attribute("size1", p_size.z / 2.0);
    // A plane's third parameter is MuJoCo's grid spacing, not a size.
    if (get_shape() != SHAPE_PLANE) {
        set_attribute("size2", p_size.y / 2.0);
    }
    changed();
}

Vector3 MujocoGeom::get_size() const
{
    double half[3];
    get_half_size(half);
    return extents_from_half(get_shape(), half);
}

void MujocoGeom::set_density(double p_density)
{
    ERR_FAIL_COND_MSG(p_density < 0.0, "Density must not be negative.");
    set_attribute("density", p_density);
    changed();
}

double MujocoGeom::get_density() const { return get_attribute("density"); }

void MujocoGeom::set_mass(double p_mass)
{
    ERR_FAIL_COND_MSG(p_mass < 0.0, "Mass must not be negative.");
    set_attribute("mass", p_mass);
    changed();
}

double MujocoGeom::get_mass() const { return get_attribute("mass"); }

void MujocoGeom::set_friction(const Vector3 &p_friction)
{
    ERR_FAIL_COND_MSG(p_friction.x < 0 || p_friction.y < 0 || p_friction.z < 0, "Friction must not be negative.");
    set_attribute("friction", p_friction);
    changed();
}

Vector3 MujocoGeom::get_friction() const { return get_attribute("friction"); }

void MujocoGeom::set_collision_layer(uint32_t p_layer)
{
    set_attribute("collision_layer", p_layer);
    changed();
}

uint32_t MujocoGeom::get_collision_layer() const { return static_cast<uint32_t>(static_cast<int64_t>(get_attribute("collision_layer"))); }

void MujocoGeom::set_collision_mask(uint32_t p_mask)
{
    set_attribute("collision_mask", p_mask);
    changed();
}

uint32_t MujocoGeom::get_collision_mask() const { return static_cast<uint32_t>(static_cast<int64_t>(get_attribute("collision_mask"))); }

void MujocoGeom::set_color(const Color &p_color)
{
    set_attribute("color", p_color);
    changed();
}

Color MujocoGeom::get_color() const { return get_attribute("color"); }

void MujocoGeom::set_show_shape(bool p_show)
{
    show_shape = p_show;
    shape_instance->set_visible(show_shape); // display only; no recompile
}

bool MujocoGeom::is_showing_shape() const { return show_shape; }
