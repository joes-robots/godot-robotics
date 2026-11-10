#include "mujoco_site.h"
#include "mujoco_convert.h"

#include <algorithm>

using namespace godot;

namespace {

// The base sizes (0.05 m half-sizes) make a 0.05 m sphere, a 0.1 m box, a 0.2 m
// capsule and a 0.1 m cylinder.
const MujocoAttribute SITE_ATTRIBUTES[] = {
    { "shape", Variant::INT, { MujocoGeom::SHAPE_SPHERE } },
    { "size0", Variant::FLOAT, { 0.05 } },
    { "size1", Variant::FLOAT, { 0.05 } },
    { "size2", Variant::FLOAT, { 0.05 } },
};
const MujocoAttributeTable SITE_TABLE = MUJOCO_ATTRIBUTE_TABLE(SITE_ATTRIBUTES);

} // namespace

const MujocoAttributeTable *MujocoSite::get_attribute_table() const
{
    return &SITE_TABLE;
}

PackedStringArray MujocoSite::get_property_attributes(const StringName &p_property) const
{
    PackedStringArray keys;
    if (p_property == StringName("radius")) {
        keys.push_back("size0");
    } else if (p_property == StringName("height")) {
        keys.push_back("size1");
    } else if (p_property == StringName("size")) {
        keys.push_back("size0");
        keys.push_back("size1");
        keys.push_back("size2");
    } else {
        keys = MujocoElement::get_property_attributes(p_property);
    }
    return keys;
}

void MujocoSite::overrides_changed()
{
    notify_property_list_changed();
    changed();
}

void MujocoSite::get_half_size(double r_half[3]) const
{
    r_half[0] = get_attribute("size0");
    r_half[1] = get_attribute("size1");
    r_half[2] = get_attribute("size2");
}

mjsBody *MujocoSite::add_to_spec(mjSpec *p_spec, mjsBody *p_body, const Transform3D &p_transform, const String &p_name)
{
    mjsSite *site = mjs_addSite(p_body, get_spec_default(p_spec));
    mjs_setName(site->element, p_name.utf8().get_data());
    write_attributes(site);
    mujoco_convert::to_mujoco(p_transform, site->pos, site->quat);
    return p_body;
}

void MujocoSite::write_attributes(mjsSite *p_site) const
{
    p_site->type = static_cast<mjtGeom>(get_shape());
    get_half_size(p_site->size);
}

void MujocoSite::write_class_defaults(mjsDefault *p_default)
{
    write_attributes(p_default->site);
}

void MujocoSite::inherited_changed()
{
    MujocoElement::inherited_changed();
    update_gizmos();
}

Ref<Mesh> MujocoSite::get_shape_mesh() const
{
    double half[3];
    get_half_size(half);
    return MujocoGeom::make_shape_mesh(get_shape(), half);
}

void MujocoSite::changed()
{
    update_gizmos();
    request_compile();
}

void MujocoSite::_validate_property(PropertyInfo &p_property) const
{
    const MujocoGeom::Shape shape = get_shape();
    const bool uses_radius = shape == MujocoGeom::SHAPE_SPHERE || shape == MujocoGeom::SHAPE_CAPSULE || shape == MujocoGeom::SHAPE_CYLINDER;
    const bool uses_height = shape == MujocoGeom::SHAPE_CAPSULE || shape == MujocoGeom::SHAPE_CYLINDER;
    const bool uses_size = shape == MujocoGeom::SHAPE_BOX || shape == MujocoGeom::SHAPE_ELLIPSOID;

    if ((p_property.name == StringName("radius") && !uses_radius) ||
            (p_property.name == StringName("height") && !uses_height) ||
            (p_property.name == StringName("size") && !uses_size)) {
        p_property.usage = PROPERTY_USAGE_NONE; // hidden; the value is stored in overrides
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

    // Values are MujocoGeom.Shape (MuJoCo's mjtGeom). Attribute properties are stored
    // in "overrides", so they are editor-only here.
    ADD_PROPERTY(PropertyInfo(Variant::INT, "shape", PROPERTY_HINT_ENUM, "Sphere:2,Capsule:3,Ellipsoid:4,Cylinder:5,Box:6", PROPERTY_USAGE_EDITOR), "set_shape", "get_shape");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "radius", PROPERTY_HINT_RANGE, "0.001,10,0.001,or_greater,suffix:m", PROPERTY_USAGE_EDITOR), "set_radius", "get_radius");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "height", PROPERTY_HINT_RANGE, "0.001,10,0.001,or_greater,suffix:m", PROPERTY_USAGE_EDITOR), "set_height", "get_height");
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "size", PROPERTY_HINT_NONE, "suffix:m", PROPERTY_USAGE_EDITOR), "set_size", "get_size");
}

void MujocoSite::set_shape(MujocoGeom::Shape p_shape)
{
    ERR_FAIL_COND_MSG(p_shape != MujocoGeom::SHAPE_SPHERE && p_shape != MujocoGeom::SHAPE_CAPSULE &&
                    p_shape != MujocoGeom::SHAPE_ELLIPSOID && p_shape != MujocoGeom::SHAPE_CYLINDER && p_shape != MujocoGeom::SHAPE_BOX,
            vformat("Unsupported site shape: %d.", p_shape));
    set_attribute("shape", p_shape);
    notify_property_list_changed();
    changed();
}

MujocoGeom::Shape MujocoSite::get_shape() const { return static_cast<MujocoGeom::Shape>(static_cast<int64_t>(get_attribute("shape"))); }

void MujocoSite::set_radius(double p_radius)
{
    ERR_FAIL_COND_MSG(p_radius <= 0.0, "Radius must be positive.");
    // A capsule keeps a height set on it, as MujocoGeom does.
    const bool keep_height = get_shape() == MujocoGeom::SHAPE_CAPSULE && has_override("size1");
    const double height = get_height();
    set_attribute("size0", p_radius);
    if (keep_height) {
        set_attribute("size1", std::max(0.0, height / 2.0 - p_radius));
    }
    changed();
}

double MujocoSite::get_radius() const { return get_attribute("size0"); }

void MujocoSite::set_height(double p_height)
{
    ERR_FAIL_COND_MSG(p_height <= 0.0, "Height must be positive.");
    const double half = get_shape() == MujocoGeom::SHAPE_CAPSULE ? std::max(0.0, p_height / 2.0 - get_radius()) : p_height / 2.0;
    set_attribute("size1", half);
    changed();
}

double MujocoSite::get_height() const
{
    double half[3];
    get_half_size(half);
    return MujocoGeom::height_from_half(get_shape(), half);
}

void MujocoSite::set_size(const Vector3 &p_size)
{
    ERR_FAIL_COND_MSG(p_size.x <= 0.0 || p_size.y <= 0.0 || p_size.z <= 0.0, "Size must be positive.");
    set_attribute("size0", p_size.x / 2.0);
    set_attribute("size1", p_size.z / 2.0);
    set_attribute("size2", p_size.y / 2.0);
    changed();
}

Vector3 MujocoSite::get_size() const
{
    double half[3];
    get_half_size(half);
    return MujocoGeom::extents_from_half(get_shape(), half);
}
