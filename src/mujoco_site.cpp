#include "mujoco_site.h"
#include "mujoco_convert.h"

using namespace godot;

mjsBody *MujocoSite::add_to_spec(mjSpec * /*p_spec*/, mjsBody *p_body, const Transform3D &p_transform, const String &p_name)
{
    mjsSite *site = mjs_addSite(p_body, nullptr);
    mjs_setName(site->element, p_name.utf8().get_data());
    site->type = static_cast<mjtGeom>(shape);
    mujoco_convert::to_mujoco(p_transform, site->pos, site->quat);
    MujocoGeom::shape_size_to_mujoco(shape, radius, height, size, site->size);
    return p_body;
}

Ref<Mesh> MujocoSite::get_shape_mesh() const
{
    return MujocoGeom::make_shape_mesh(shape, radius, height, size);
}

void MujocoSite::changed()
{
    update_gizmos();
    request_compile();
}

void MujocoSite::_validate_property(PropertyInfo &p_property) const
{
    const bool uses_radius = shape == MujocoGeom::SHAPE_SPHERE || shape == MujocoGeom::SHAPE_CAPSULE || shape == MujocoGeom::SHAPE_CYLINDER;
    const bool uses_height = shape == MujocoGeom::SHAPE_CAPSULE || shape == MujocoGeom::SHAPE_CYLINDER;
    const bool uses_size = shape == MujocoGeom::SHAPE_BOX || shape == MujocoGeom::SHAPE_ELLIPSOID;

    if ((p_property.name == StringName("radius") && !uses_radius) ||
            (p_property.name == StringName("height") && !uses_height) ||
            (p_property.name == StringName("size") && !uses_size)) {
        p_property.usage = PROPERTY_USAGE_NO_EDITOR;
    }
}

void MujocoSite::_bind_methods()
{
    ClassDB::bind_method(D_METHOD("set_shape", "shape"), &MujocoSite::set_shape);
    ClassDB::bind_method(D_METHOD("get_shape"), &MujocoSite::get_shape);
    ClassDB::bind_method(D_METHOD("set_radius", "radius"), &MujocoSite::set_radius);
    ClassDB::bind_method(D_METHOD("get_radius"), &MujocoSite::get_radius);
    ClassDB::bind_method(D_METHOD("set_height", "height"), &MujocoSite::set_height);
    ClassDB::bind_method(D_METHOD("get_height"), &MujocoSite::get_height);
    ClassDB::bind_method(D_METHOD("set_size", "size"), &MujocoSite::set_size);
    ClassDB::bind_method(D_METHOD("get_size"), &MujocoSite::get_size);

    // Values are MujocoGeom.Shape (MuJoCo's mjtGeom).
    ADD_PROPERTY(PropertyInfo(Variant::INT, "shape", PROPERTY_HINT_ENUM, "Sphere:2,Capsule:3,Ellipsoid:4,Cylinder:5,Box:6"), "set_shape", "get_shape");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "radius", PROPERTY_HINT_RANGE, "0.001,10,0.001,or_greater,suffix:m"), "set_radius", "get_radius");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "height", PROPERTY_HINT_RANGE, "0.001,10,0.001,or_greater,suffix:m"), "set_height", "get_height");
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "size", PROPERTY_HINT_NONE, "suffix:m"), "set_size", "get_size");
}

void MujocoSite::set_shape(MujocoGeom::Shape p_shape)
{
    ERR_FAIL_COND_MSG(p_shape != MujocoGeom::SHAPE_SPHERE && p_shape != MujocoGeom::SHAPE_CAPSULE &&
                    p_shape != MujocoGeom::SHAPE_ELLIPSOID && p_shape != MujocoGeom::SHAPE_CYLINDER && p_shape != MujocoGeom::SHAPE_BOX,
            vformat("Unsupported site shape: %d.", p_shape));
    shape = p_shape;
    notify_property_list_changed();
    changed();
}

MujocoGeom::Shape MujocoSite::get_shape() const { return shape; }

void MujocoSite::set_radius(double p_radius)
{
    ERR_FAIL_COND_MSG(p_radius <= 0.0, "Radius must be positive.");
    radius = p_radius;
    changed();
}

double MujocoSite::get_radius() const { return radius; }

void MujocoSite::set_height(double p_height)
{
    ERR_FAIL_COND_MSG(p_height <= 0.0, "Height must be positive.");
    height = p_height;
    changed();
}

double MujocoSite::get_height() const { return height; }

void MujocoSite::set_size(const Vector3 &p_size)
{
    ERR_FAIL_COND_MSG(p_size.x <= 0.0 || p_size.y <= 0.0 || p_size.z <= 0.0, "Size must be positive.");
    size = p_size;
    changed();
}

Vector3 MujocoSite::get_size() const { return size; }
