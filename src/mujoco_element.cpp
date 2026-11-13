#include "mujoco_element.h"

#include <godot_cpp/variant/utility_functions.hpp>

#include "mujoco_world.h"

using namespace godot;

MujocoElement::MujocoElement()
{
    // Ask Godot to send NOTIFICATION_LOCAL_TRANSFORM_CHANGED when this node is
    // moved; it doesn't by default.
    set_notify_local_transform(true);
}

void MujocoElement::_bind_methods()
{
    ClassDB::bind_method(D_METHOD("get_mujoco_name"), &MujocoElement::get_mujoco_name);

    ClassDB::bind_method(D_METHOD("has_override", "name"), &MujocoElement::has_override);
    ClassDB::bind_method(D_METHOD("clear_override", "name"), &MujocoElement::clear_override);
    ClassDB::bind_method(D_METHOD("get_overrides"), &MujocoElement::get_overrides);
    ClassDB::bind_method(D_METHOD("set_overrides", "overrides"), &MujocoElement::set_overrides);
    // The one property that is saved. NO_EDITOR means "stored, but not shown in
    // the inspector": the inspector shows the individual attributes instead.
    ADD_PROPERTY(PropertyInfo(Variant::DICTIONARY, "overrides", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NO_EDITOR), "set_overrides", "get_overrides");
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

String MujocoElement::get_mujoco_name() const
{
    const MujocoWorld *world = get_world();
    if (!world) {
        return String();
    }
    // get_path_to isn't declared const in godot-cpp, but it doesn't modify the node.
    return String(world->get_path_to(const_cast<MujocoElement *>(this)));
}

Transform3D MujocoElement::get_spec_transform() const
{
    return get_transform();
}

void MujocoElement::request_recompile()
{
    if (MujocoWorld *world = get_world()) {
        world->request_recompile();
    }
}

// Everything here changes the model's structure, a name in it, or a pose in it,
// so the world has to rebuild. The request is cheap: the world compiles once
// per frame however many requests it gets.
void MujocoElement::_notification(int p_what)
{
    switch (p_what) {
        case NOTIFICATION_ENTER_TREE:
        case NOTIFICATION_EXIT_TREE:
        case NOTIFICATION_PATH_RENAMED:
        case NOTIFICATION_CHILD_ORDER_CHANGED:
            request_recompile();
            break;
        case NOTIFICATION_LOCAL_TRANSFORM_CHANGED:
            if (!applying_simulation_pose) {
                request_recompile();
            }
            break;
    }
}

// ---- Attributes ----

// By default an element has no attributes.
const MujocoAttribute *MujocoElement::get_attribute_table(int &r_count) const
{
    r_count = 0;
    return nullptr;
}

Variant MujocoElement::get_inherited_value(int p_index) const
{
    return Variant();
}

int MujocoElement::find_attribute(const StringName &p_name) const
{
    int count = 0;
    const MujocoAttribute *table = get_attribute_table(count);
    const String name = p_name;
    for (int i = 0; i < count; i++) {
        if (name == table[i].name) {
            return i;
        }
    }
    return -1;
}

bool MujocoElement::get_override(int p_index, Variant &r_value) const
{
    int count = 0;
    const MujocoAttribute *table = get_attribute_table(count);
    const String name = table[p_index].name;
    if (!overrides.has(name)) {
        return false;
    }
    r_value = overrides[name];
    return true;
}

Variant MujocoElement::get_attribute(int p_index) const
{
    Variant value;
    if (!get_override(p_index, value)) {
        value = get_inherited_value(p_index);
    }
    return value;
}

void MujocoElement::attributes_changed()
{
}

bool MujocoElement::has_override(const StringName &p_name) const
{
    return overrides.has(String(p_name));
}

void MujocoElement::clear_override(const StringName &p_name)
{
    if (overrides.erase(String(p_name))) {
        request_recompile();
        attributes_changed();
        // Refreshes the inspector's revert arrow.
        notify_property_list_changed();
    }
}

Dictionary MujocoElement::get_overrides() const
{
    return overrides.duplicate();
}

// Replaces all overrides. This is what loading a scene calls. Each entry goes
// through _set, so it gets the same checking as setting the property directly.
void MujocoElement::set_overrides(const Dictionary &p_overrides)
{
    overrides.clear();
    const Array names = p_overrides.keys();
    for (int i = 0; i < names.size(); i++) {
        const StringName name = names[i];
        if (find_attribute(name) < 0) {
            WARN_PRINT(String("Ignoring unknown attribute '") + String(name) + String("' on ") + get_class() + String("."));
            continue;
        }
        _set(name, p_overrides[names[i]]);
    }
    request_recompile();
    attributes_changed();
}

// Shows each attribute in the inspector. PROPERTY_USAGE_EDITOR means "shown, but
// not saved": the values are saved through `overrides` instead.
void MujocoElement::get_attribute_property_list(List<PropertyInfo> *r_list) const
{
    int count = 0;
    const MujocoAttribute *table = get_attribute_table(count);
    for (int i = 0; i < count; i++) {
        r_list->push_back(PropertyInfo(table[i].type, table[i].name, table[i].hint, table[i].hint_string, PROPERTY_USAGE_EDITOR));
    }
}

// Reading an attribute gives its override if it has one, else what it inherits.
bool MujocoElement::_get(const StringName &p_name, Variant &r_value) const
{
    const int index = find_attribute(p_name);
    if (index < 0) {
        return false;
    }
    r_value = get_attribute(index);
    return true;
}

// Writing an attribute stores an override. Writing null removes it.
bool MujocoElement::_set(const StringName &p_name, const Variant &p_value)
{
    const int index = find_attribute(p_name);
    if (index < 0) {
        return false;
    }
    const String name = p_name;

    if (p_value.get_type() == Variant::NIL) {
        clear_override(p_name);
        return true;
    }

    // Store the value as the attribute's own type, e.g. an int given for a
    // float attribute becomes a float.
    int count = 0;
    const Variant::Type type = get_attribute_table(count)[index].type;
    ERR_FAIL_COND_V_MSG(!Variant::can_convert_strict(p_value.get_type(), type), true,
            name + String(" must be of type ") + Variant::get_type_name(type) + String("."));

    const bool was_overridden = overrides.has(name);
    overrides[name] = UtilityFunctions::type_convert(p_value, type);
    request_recompile();
    attributes_changed();
    if (!was_overridden) {
        // The revert arrow has just become available.
        notify_property_list_changed();
    }
    return true;
}

// An attribute can be reverted when it is overridden. Reverting assigns the
// value given here, null, which _set treats as "remove the override".
bool MujocoElement::_property_can_revert(const StringName &p_name) const
{
    return find_attribute(p_name) >= 0 && has_override(p_name);
}

bool MujocoElement::_property_get_revert(const StringName &p_name, Variant &r_value) const
{
    if (find_attribute(p_name) < 0) {
        return false;
    }
    r_value = Variant();
    return true;
}
