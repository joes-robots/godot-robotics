#include "mujoco_default.h"
#include "mujoco_actuator.h"
#include "mujoco_geom.h"
#include "mujoco_joint.h"
#include "mujoco_site.h"
#include "mujoco_tendon.h"
#include "mujoco_world.h"

#include <godot_cpp/core/memory.hpp>
#include <godot_cpp/variant/callable_method_pointer.hpp>

using namespace godot;

MujocoDefault::MujocoDefault()
{
    sections[0] = memnew(MujocoJoint);
    sections[1] = memnew(MujocoGeom);
    sections[2] = memnew(MujocoSite);
    sections[3] = memnew(MujocoTendon);
    sections[4] = memnew(MujocoActuator);
    for (MujocoElement *element : sections) {
        element->class_owner = this;
        // A section's visible properties depend on its values (e.g. the geom shape).
        element->connect("property_list_changed", callable_mp(static_cast<Object *>(this), &Object::notify_property_list_changed));
    }
}

MujocoDefault::~MujocoDefault()
{
    for (MujocoElement *&element : sections) {
        memdelete(element);
        element = nullptr;
    }
}

MujocoWorld *MujocoDefault::get_world() const
{
    for (Node *node = get_parent(); node; node = node->get_parent()) {
        if (MujocoWorld *world = Object::cast_to<MujocoWorld>(node)) {
            return world;
        }
    }
    return nullptr;
}

bool MujocoDefault::is_main() const
{
    return get_name() == StringName("main") && Object::cast_to<MujocoWorld>(get_parent());
}

MujocoDefault *MujocoDefault::get_parent_class() const
{
    if (MujocoDefault *parent = Object::cast_to<MujocoDefault>(get_parent())) {
        return parent;
    }
    if (is_main()) {
        return nullptr;
    }
    const MujocoWorld *world = get_world();
    return world ? world->get_main_class() : nullptr;
}

MujocoElement *MujocoDefault::get_section_element(const char *p_section) const
{
    for (MujocoElement *element : sections) {
        if (strcmp(element->get_class_section(), p_section) == 0) {
            return element;
        }
    }
    return nullptr;
}

void MujocoDefault::write_to_spec(mjsDefault *p_default) const
{
    for (MujocoElement *element : sections) {
        element->write_class_defaults(p_default);
    }
}

void MujocoDefault::section_changed()
{
    if (MujocoWorld *world = get_world()) {
        world->default_classes_changed();
    }
}

MujocoElement *MujocoDefault::split_property(const StringName &p_name, StringName &r_property) const
{
    const String name = p_name;
    const int slash = name.find("/");
    if (slash < 0) {
        return nullptr;
    }
    MujocoElement *element = get_section_element(name.substr(0, slash).utf8().get_data());
    if (!element) {
        return nullptr;
    }
    r_property = name.substr(slash + 1);
    if (element->get_property_attributes(r_property).is_empty()) {
        return nullptr;
    }
    return element;
}

bool MujocoDefault::_set(const StringName &p_name, const Variant &p_value)
{
    StringName property;
    MujocoElement *element = split_property(p_name, property);
    if (!element) {
        return false;
    }
    // Through the element's own setter (validation, size views) or revert (null).
    element->set(property, p_value);
    return true;
}

bool MujocoDefault::_get(const StringName &p_name, Variant &r_ret) const
{
    StringName property;
    const MujocoElement *element = split_property(p_name, property);
    if (!element) {
        return false;
    }
    r_ret = element->get(property);
    return true;
}

void MujocoDefault::_get_property_list(List<PropertyInfo> *p_list) const
{
    for (const MujocoElement *element : sections) {
        const String section = element->get_class_section();
        const TypedArray<Dictionary> properties = element->get_property_list();
        for (int64_t i = 0; i < properties.size(); i++) {
            const Dictionary info = properties[i];
            const StringName name = info["name"];
            const uint32_t usage = static_cast<uint32_t>(static_cast<int64_t>(info["usage"]));
            // The element's class-able properties that its inspector shows.
            if (!(usage & PROPERTY_USAGE_EDITOR) || element->get_property_attributes(name).is_empty()) {
                continue;
            }
            // Editor-only: the values are saved through "overrides".
            p_list->push_back(PropertyInfo(static_cast<Variant::Type>(static_cast<int64_t>(info["type"])), section + String("/") + String(name),
                    static_cast<PropertyHint>(static_cast<int64_t>(info["hint"])), info["hint_string"], PROPERTY_USAGE_EDITOR, info["class_name"]));
        }
    }
}

bool MujocoDefault::_property_can_revert(const StringName &p_name) const
{
    StringName property;
    const MujocoElement *element = split_property(p_name, property);
    return element && element->property_can_revert(property);
}

bool MujocoDefault::_property_get_revert(const StringName &p_name, Variant &r_property) const
{
    StringName property;
    const MujocoElement *element = split_property(p_name, property);
    if (!element) {
        return false;
    }
    r_property = element->property_get_revert(property);
    return true;
}

void MujocoDefault::set_overrides(const Dictionary &p_overrides)
{
    for (MujocoElement *element : sections) {
        const String section = element->get_class_section();
        element->set_overrides(p_overrides.get(section, Dictionary()));
    }
}

Dictionary MujocoDefault::get_overrides() const
{
    Dictionary result;
    for (const MujocoElement *element : sections) {
        const Dictionary overrides = element->get_overrides();
        if (!overrides.is_empty()) {
            result[String(element->get_class_section())] = overrides;
        }
    }
    return result;
}

PackedStringArray MujocoDefault::_get_configuration_warnings() const
{
    PackedStringArray warnings = Node::_get_configuration_warnings();
    const MujocoWorld *world = get_world();
    if (!world) {
        warnings.push_back("Must be a descendant of a MujocoWorld node.");
        return warnings;
    }
    if (!Object::cast_to<MujocoWorld>(get_parent()) && !Object::cast_to<MujocoDefault>(get_parent())) {
        warnings.push_back("Must be a child of the MujocoWorld or of another MujocoDefault.");
    }
    if (get_name() == StringName("main") && !is_main()) {
        warnings.push_back("Only a class directly under the MujocoWorld can be \"main\"; MuJoCo has one root class.");
    }
    if (world->find_default_class(get_name()) != this) {
        warnings.push_back(vformat("Another class in this world is also named \"%s\"; class names must be unique.", get_name()));
    }
    return warnings;
}

void MujocoDefault::_notification(int p_what)
{
    switch (p_what) {
        case NOTIFICATION_ENTER_TREE:
        case NOTIFICATION_EXIT_TREE:
        case NOTIFICATION_PATH_RENAMED:
        case NOTIFICATION_CHILD_ORDER_CHANGED:
            // The parent chain is still intact on exit, so the world can be found.
            section_changed();
            break;
    }
}

void MujocoDefault::_bind_methods()
{
    ClassDB::bind_method(D_METHOD("set_overrides", "overrides"), &MujocoDefault::set_overrides);
    ClassDB::bind_method(D_METHOD("get_overrides"), &MujocoDefault::get_overrides);
    ClassDB::bind_method(D_METHOD("is_main"), &MujocoDefault::is_main);
    ClassDB::bind_method(D_METHOD("get_parent_class"), &MujocoDefault::get_parent_class);
    ADD_PROPERTY(PropertyInfo(Variant::DICTIONARY, "overrides", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NO_EDITOR), "set_overrides", "get_overrides");
}
