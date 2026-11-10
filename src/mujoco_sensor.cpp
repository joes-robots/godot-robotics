#include "mujoco_sensor.h"
#include "mujoco_actuator.h"
#include "mujoco_body.h"
#include "mujoco_convert.h"
#include "mujoco_joint.h"
#include "mujoco_site.h"
#include "mujoco_world.h"

#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/variant/quaternion.hpp>

using namespace godot;

namespace {

enum TargetKind {
    TARGET_JOINT,    // hinge or slide MujocoJoint
    TARGET_ACTUATOR, // MujocoActuator
    TARGET_FRAME,    // MujocoSite or MujocoBody
    TARGET_SITE,     // MujocoSite
};

enum Output {
    OUTPUT_SCALAR,        // float
    OUTPUT_GLOBAL_POINT,  // Vector3, global position
    OUTPUT_GLOBAL_QUAT,   // Quaternion, global orientation
    OUTPUT_GLOBAL_VECTOR, // Vector3, global direction
    OUTPUT_LOCAL_VECTOR,  // Vector3, in the site's local frame
};

struct SensorInfo {
    mjtSensor mujoco_type;
    TargetKind target;
    Output output;
};

// Indexed by MujocoSensor::Type.
const SensorInfo SENSOR_INFO[] = {
    { mjSENS_JOINTPOS, TARGET_JOINT, OUTPUT_SCALAR },
    { mjSENS_JOINTVEL, TARGET_JOINT, OUTPUT_SCALAR },
    { mjSENS_ACTUATORFRC, TARGET_ACTUATOR, OUTPUT_SCALAR },
    { mjSENS_FRAMEPOS, TARGET_FRAME, OUTPUT_GLOBAL_POINT },
    { mjSENS_FRAMEQUAT, TARGET_FRAME, OUTPUT_GLOBAL_QUAT },
    { mjSENS_FRAMELINVEL, TARGET_FRAME, OUTPUT_GLOBAL_VECTOR },
    { mjSENS_FRAMEANGVEL, TARGET_FRAME, OUTPUT_GLOBAL_VECTOR },
    { mjSENS_ACCELEROMETER, TARGET_SITE, OUTPUT_LOCAL_VECTOR },
    { mjSENS_GYRO, TARGET_SITE, OUTPUT_LOCAL_VECTOR },
    { mjSENS_VELOCIMETER, TARGET_SITE, OUTPUT_LOCAL_VECTOR },
    { mjSENS_TOUCH, TARGET_SITE, OUTPUT_SCALAR },
};

} // namespace

Node3D *MujocoSensor::find_target(String &r_problem) const
{
    const MujocoWorld *world = get_world();
    if (!world) {
        r_problem = "Not inside a MujocoWorld.";
        return nullptr;
    }
    Node *node = get_node_or_null(target);
    MujocoElement *element = Object::cast_to<MujocoElement>(node);
    const SensorInfo &info = SENSOR_INFO[type];
    switch (info.target) {
        case TARGET_JOINT: {
            const MujocoJoint *joint = Object::cast_to<MujocoJoint>(node);
            if (!joint) {
                r_problem = "\"target\" must point to a MujocoJoint.";
                return nullptr;
            }
            if (joint->get_type() != MujocoJoint::TYPE_HINGE && joint->get_type() != MujocoJoint::TYPE_SLIDE) {
                r_problem = "Only hinge and slide joints have a single position and velocity.";
                return nullptr;
            }
        } break;
        case TARGET_ACTUATOR: {
            const MujocoActuator *actuator = Object::cast_to<MujocoActuator>(node);
            if (!actuator) {
                r_problem = "\"target\" must point to a MujocoActuator.";
                return nullptr;
            }
            String actuator_problem;
            if (!actuator->find_joint(actuator_problem)) {
                r_problem = "The target actuator is left out of the model: " + actuator_problem;
                return nullptr;
            }
        } break;
        case TARGET_FRAME:
            if (!Object::cast_to<MujocoSite>(node) && !Object::cast_to<MujocoBody>(node)) {
                r_problem = "\"target\" must point to a MujocoSite or a MujocoBody.";
                return nullptr;
            }
            break;
        case TARGET_SITE:
            if (!Object::cast_to<MujocoSite>(node)) {
                r_problem = "\"target\" must point to a MujocoSite.";
                return nullptr;
            }
            break;
    }
    if (element->get_world() != world) {
        r_problem = "The target belongs to a different MujocoWorld.";
        return nullptr;
    }
    return element;
}

mjsBody *MujocoSensor::add_to_spec(mjSpec *p_spec, mjsBody *p_body, const Transform3D & /*p_transform*/, const String &p_name)
{
    String problem;
    Node3D *target_node = find_target(problem);
    if (!target_node) {
        // Like an actuator without a joint: left out rather than failing the model.
        if (!Engine::get_singleton()->is_editor_hint()) {
            WARN_PRINT(vformat("MuJoCo sensor '%s' is left out of the model: %s", p_name, problem));
        }
        return p_body;
    }

    const SensorInfo &info = SENSOR_INFO[type];
    mjsSensor *sensor = mjs_addSensor(p_spec);
    mjs_setName(sensor->element, p_name.utf8().get_data());
    sensor->type = info.mujoco_type;
    switch (info.target) {
        case TARGET_JOINT:
            sensor->objtype = mjOBJ_JOINT;
            break;
        case TARGET_ACTUATOR:
            sensor->objtype = mjOBJ_ACTUATOR;
            break;
        case TARGET_FRAME:
            // A body is measured at its origin (MuJoCo's xbody), where its node is.
            sensor->objtype = Object::cast_to<MujocoSite>(target_node) ? mjOBJ_SITE : mjOBJ_XBODY;
            break;
        case TARGET_SITE:
            sensor->objtype = mjOBJ_SITE;
            break;
    }
    // Element names are node paths relative to the world.
    mjs_setString(sensor->objname, String(get_world()->get_path_to(target_node)).utf8().get_data());
    return p_body;
}

Variant MujocoSensor::get_value()
{
    const int id = get_model_id(mjOBJ_SENSOR);
    ERR_FAIL_COND_V_MSG(id < 0, Variant(), "This sensor is not part of a compiled MuJoCo model.");
    MujocoWorld *world = get_world();
    const mjModel *model = world->get_model();
    const mjtNum *value = world->get_data()->sensordata + model->sensor_adr[id];

    switch (SENSOR_INFO[type].output) {
        case OUTPUT_SCALAR:
            return value[0];
        case OUTPUT_GLOBAL_POINT:
            return world->get_global_transform().xform(mujoco_convert::to_godot(value));
        case OUTPUT_GLOBAL_QUAT: {
            const mjtNum origin[3] = { 0, 0, 0 };
            const Basis local = mujoco_convert::to_godot(origin, value).basis;
            return (world->get_global_basis().orthonormalized() * local).get_rotation_quaternion();
        }
        case OUTPUT_GLOBAL_VECTOR:
            return world->mujoco_to_global_vector(value);
        case OUTPUT_LOCAL_VECTOR:
            // MuJoCo's site axes x, y, z are the site node's local x, -z, y.
            return mujoco_convert::to_godot(value);
    }
    return Variant();
}

PackedStringArray MujocoSensor::_get_configuration_warnings() const
{
    PackedStringArray warnings = MujocoElement::_get_configuration_warnings();
    String problem;
    if (get_world() && !find_target(problem)) {
        warnings.push_back(problem + " This sensor is left out of the model.");
    }
    return warnings;
}

void MujocoSensor::changed()
{
    update_configuration_warnings();
    request_compile();
}

void MujocoSensor::_bind_methods()
{
    BIND_ENUM_CONSTANT(TYPE_JOINT_POSITION);
    BIND_ENUM_CONSTANT(TYPE_JOINT_VELOCITY);
    BIND_ENUM_CONSTANT(TYPE_ACTUATOR_FORCE);
    BIND_ENUM_CONSTANT(TYPE_FRAME_POSITION);
    BIND_ENUM_CONSTANT(TYPE_FRAME_ORIENTATION);
    BIND_ENUM_CONSTANT(TYPE_FRAME_LINEAR_VELOCITY);
    BIND_ENUM_CONSTANT(TYPE_FRAME_ANGULAR_VELOCITY);
    BIND_ENUM_CONSTANT(TYPE_ACCELEROMETER);
    BIND_ENUM_CONSTANT(TYPE_GYRO);
    BIND_ENUM_CONSTANT(TYPE_VELOCIMETER);
    BIND_ENUM_CONSTANT(TYPE_TOUCH);

    ClassDB::bind_method(D_METHOD("set_type", "type"), &MujocoSensor::set_type);
    ClassDB::bind_method(D_METHOD("get_type"), &MujocoSensor::get_type);
    ClassDB::bind_method(D_METHOD("set_target", "target"), &MujocoSensor::set_target);
    ClassDB::bind_method(D_METHOD("get_target"), &MujocoSensor::get_target);
    ClassDB::bind_method(D_METHOD("get_value"), &MujocoSensor::get_value);

    ADD_PROPERTY(PropertyInfo(Variant::INT, "type", PROPERTY_HINT_ENUM,
                         "Joint Position,Joint Velocity,Actuator Force,Frame Position,Frame Orientation,"
                         "Frame Linear Velocity,Frame Angular Velocity,Accelerometer,Gyro,Velocimeter,Touch"),
            "set_type", "get_type");
    ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "target", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "MujocoJoint,MujocoActuator,MujocoSite,MujocoBody"), "set_target", "get_target");
}

void MujocoSensor::set_type(Type p_type)
{
    ERR_FAIL_COND_MSG(p_type < TYPE_JOINT_POSITION || p_type > TYPE_TOUCH, vformat("Unsupported sensor type: %d.", p_type));
    type = p_type;
    changed();
}

MujocoSensor::Type MujocoSensor::get_type() const { return type; }

void MujocoSensor::set_target(const NodePath &p_target)
{
    target = p_target;
    changed();
}

NodePath MujocoSensor::get_target() const { return target; }
