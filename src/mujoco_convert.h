#pragma once

// Conversions between Godot's and MuJoCo's coordinate conventions.
//
// Both are right-handed, but Godot is Y-up and MuJoCo is Z-up:
//
//            Godot   MuJoCo
//   right      +x      +x
//   up         +y      +z
//   forward    -z      +y
//
// So a Godot vector (x, y, z) is the MuJoCo vector (x, -z, y),
// and a MuJoCo vector (x, y, z) is the Godot vector (x, z, -y).
//
// Everything that crosses between the two libraries should go through this
// header, so the convention is written down in exactly one place.

#include <godot_cpp/variant/vector3.hpp>
#include <mujoco/mujoco.h>

namespace mujoco_convert {

// Writes a Godot vector into a MuJoCo 3-vector.
inline void to_mujoco(const godot::Vector3 &p_godot, mjtNum r_mujoco[3]) {
    r_mujoco[0] = p_godot.x;
    r_mujoco[1] = -p_godot.z;
    r_mujoco[2] = p_godot.y;
}

// Reads a MuJoCo 3-vector as a Godot vector.
inline godot::Vector3 to_godot(const mjtNum p_mujoco[3]) {
    return godot::Vector3(p_mujoco[0], p_mujoco[2], -p_mujoco[1]);
}

} // namespace mujoco_convert
