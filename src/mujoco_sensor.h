#pragma once

#include "mujoco_element.h"

#include <godot_cpp/variant/node_path.hpp>

// A MuJoCo sensor. It measures the node `target` points to (by default its
// parent, ".."); the node's own transform is not used. get_value() returns the
// reading in Godot conventions:
//
//   Joint Position / Velocity  (target: hinge or slide MujocoJoint)  float, like
//       MujocoJoint.get_joint_position() / get_joint_velocity()
//   Actuator Force             (target: MujocoActuator)  float, like MujocoActuator.get_force()
//   Frame Position / Orientation / Linear Velocity / Angular Velocity
//                              (target: MujocoSite or MujocoBody)  Vector3 or Quaternion,
//       in global coordinates; a body is measured at its origin
//   Accelerometer / Gyro / Velocimeter  (target: MujocoSite)  Vector3 in the site's
//       local frame. The accelerometer includes gravity's reaction: at rest it reads
//       9.81 m/s^2 up. Bodies welded to the world (no joints) are static in MuJoCo,
//       so sensors on them read 0 here.
//   Touch                      (target: MujocoSite)  float, the total normal force of
//       the contacts inside the site's shape, in newtons
//   Tendon Position / Velocity (target: MujocoTendon)  float, like
//       MujocoTendon.get_length() / get_velocity()
//
// Readings are updated after every physics frame's steps and after step(); in a
// pre_step handler they can be one step old.
class MujocoSensor : public MujocoElement {
    GDCLASS(MujocoSensor, MujocoElement);

public:
    enum Type {
        TYPE_JOINT_POSITION,
        TYPE_JOINT_VELOCITY,
        TYPE_ACTUATOR_FORCE,
        TYPE_FRAME_POSITION,
        TYPE_FRAME_ORIENTATION,
        TYPE_FRAME_LINEAR_VELOCITY,
        TYPE_FRAME_ANGULAR_VELOCITY,
        TYPE_ACCELEROMETER,
        TYPE_GYRO,
        TYPE_VELOCIMETER,
        TYPE_TOUCH,
        TYPE_TENDON_POSITION,
        TYPE_TENDON_VELOCITY,
    };

    mjsBody *add_to_spec(mjSpec *p_spec, mjsBody *p_body, const godot::Transform3D &p_transform, const godot::String &p_name) override;
    godot::PackedStringArray _get_configuration_warnings() const override;

    void set_type(Type p_type);
    Type get_type() const;
    void set_target(const godot::NodePath &p_target);
    godot::NodePath get_target() const;

    // The current reading (see the class comment), or null with an error if the
    // sensor isn't part of a compiled model.
    godot::Variant get_value();

protected:
    static void _bind_methods();

private:
    void changed();
    // The node this sensor measures, or nullptr with the reason in r_problem.
    godot::Node3D *find_target(godot::String &r_problem) const;

    Type type = TYPE_JOINT_POSITION;
    godot::NodePath target = godot::NodePath("..");
};

VARIANT_ENUM_CAST(MujocoSensor::Type);
