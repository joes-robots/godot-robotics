#include "mujoco_element.h"
#include "mujoco_world.h"

#include <godot_cpp/classes/engine.hpp>

using namespace godot;

mjsBody *MujocoElement::add_to_spec(mjSpec * /*p_spec*/, mjsBody *p_body, const Transform3D & /*p_transform*/, const String & /*p_name*/)
{
    return p_body;
}

Transform3D MujocoElement::get_model_transform()
{
    return get_transform();
}

MujocoWorld *MujocoElement::get_world() const
{
    for (Node *node = get_parent(); node; node = node->get_parent()) {
        if (MujocoWorld *world = Object::cast_to<MujocoWorld>(node)) {
            return world;
        }
    }
    return nullptr;
}

void MujocoElement::request_compile() const
{
    if (MujocoWorld *world = get_world()) {
        world->request_compile();
    }
}

int MujocoElement::get_model_id(mjtObj p_type)
{
    MujocoWorld *world = get_world();
    if (!world) {
        return -1;
    }
    const mjModel *model = world->get_model();
    if (!model) {
        return -1;
    }
    // Element names are node paths relative to the world (see MujocoWorld::add_subtree).
    if (cached_generation != world->get_model_generation()) {
        cached_id = mj_name2id(model, p_type, String(world->get_path_to(this)).utf8().get_data());
        cached_generation = world->get_model_generation();
    }
    return cached_id;
}

Variant MujocoElement::get_inherited(const MujocoAttribute &p_attribute) const
{
    return p_attribute.get_base();
}

Variant MujocoElement::get_attribute(const StringName &p_key) const
{
    if (overrides.has(p_key)) {
        return overrides[p_key];
    }
    const MujocoAttributeTable *table = get_attribute_table();
    const MujocoAttribute *attribute = table ? table->find(p_key) : nullptr;
    ERR_FAIL_NULL_V_MSG(attribute, Variant(), vformat("%s has no attribute '%s'.", get_class(), p_key));
    return get_inherited(*attribute);
}

bool MujocoElement::has_override(const StringName &p_key) const
{
    return overrides.has(p_key);
}

bool MujocoElement::set_attribute(const StringName &p_key, const Variant &p_value)
{
    const MujocoAttributeTable *table = get_attribute_table();
    const MujocoAttribute *attribute = table ? table->find(p_key) : nullptr;
    ERR_FAIL_NULL_V_MSG(attribute, false, vformat("%s has no attribute '%s'.", get_class(), p_key));
    Variant value;
    ERR_FAIL_COND_V_MSG(!attribute->convert(p_value, value), false,
            vformat("%s attribute '%s' needs a %s, not a %s.", get_class(), p_key, Variant::get_type_name(attribute->type), Variant::get_type_name(p_value.get_type())));
    // Keys are Strings so the saved dictionary reads like the inspector.
    overrides[String(p_key)] = value;
    return true;
}

void MujocoElement::set_overrides(const Dictionary &p_overrides)
{
    overrides.clear();
    const Array keys = p_overrides.keys();
    for (int64_t i = 0; i < keys.size(); i++) {
        set_attribute(keys[i], p_overrides[keys[i]]);
    }
    overrides_changed();
}

Dictionary MujocoElement::get_overrides() const
{
    return overrides.duplicate();
}

PackedStringArray MujocoElement::get_property_attributes(const StringName &p_property) const
{
    PackedStringArray keys;
    const MujocoAttributeTable *table = get_attribute_table();
    if (table && table->find(p_property)) {
        keys.push_back(p_property);
    }
    return keys;
}

bool MujocoElement::_set(const StringName &p_name, const Variant &p_value)
{
    // Only null is handled here; other values go on to the property's setter.
    if (p_value.get_type() != Variant::NIL) {
        return false;
    }
    const PackedStringArray keys = get_property_attributes(p_name);
    if (keys.is_empty()) {
        return false;
    }
    for (int64_t i = 0; i < keys.size(); i++) {
        overrides.erase(keys[i]);
    }
    overrides_changed();
    return true;
}

bool MujocoElement::_property_can_revert(const StringName &p_name) const
{
    return !get_property_attributes(p_name).is_empty();
}

bool MujocoElement::_property_get_revert(const StringName &p_name, Variant &r_property) const
{
    const PackedStringArray keys = get_property_attributes(p_name);
    if (keys.is_empty()) {
        return false;
    }
    for (int64_t i = 0; i < keys.size(); i++) {
        if (overrides.has(keys[i])) {
            // Reverting assigns null, which removes the overrides (see _set).
            r_property = Variant();
            return true;
        }
    }
    // Not overridden: the revert value is the current (inherited) value, so the
    // inspector shows no revert arrow.
    r_property = get(p_name);
    return true;
}

void MujocoElement::_bind_methods()
{
    ClassDB::bind_method(D_METHOD("set_overrides", "overrides"), &MujocoElement::set_overrides);
    ClassDB::bind_method(D_METHOD("get_overrides"), &MujocoElement::get_overrides);
    ADD_PROPERTY(PropertyInfo(Variant::DICTIONARY, "overrides", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NO_EDITOR), "set_overrides", "get_overrides");
}

PackedStringArray MujocoElement::_get_configuration_warnings() const
{
    PackedStringArray warnings = Node3D::_get_configuration_warnings();

    MujocoWorld *world = get_world();
    if (!world) {
        warnings.push_back("Must be a descendant of a MujocoWorld node.");
    } else {
        for (Node *node = get_parent(); node != world; node = node->get_parent()) {
            if (!Object::cast_to<Node3D>(node)) {
                warnings.push_back(vformat("All nodes between this node and its MujocoWorld must be Node3D, but '%s' is not. This node is ignored.", node->get_name()));
                break;
            }
        }
    }

    const Vector3 scale = get_scale();
    if (!scale.is_equal_approx(Vector3(1, 1, 1))) {
        warnings.push_back("MuJoCo does not support scale; it is ignored. Use this node's size properties instead.");
    }
    return warnings;
}

void MujocoElement::_notification(int p_what)
{
    switch (p_what) {
        case NOTIFICATION_ENTER_TREE:
            // In the editor, moving this node or any ancestor changes the model.
            // At runtime the simulation drives transforms, so don't listen.
            set_notify_transform(Engine::get_singleton()->is_editor_hint());
            update_configuration_warnings();
            request_compile();
            break;
        case NOTIFICATION_EXIT_TREE:
            // The parent chain is still intact here, so the world can be found.
            request_compile();
            break;
        case NOTIFICATION_TRANSFORM_CHANGED:
            update_configuration_warnings();
            request_compile();
            break;
        case NOTIFICATION_PATH_RENAMED:
        case NOTIFICATION_CHILD_ORDER_CHANGED:
            request_compile();
            break;
    }
}
