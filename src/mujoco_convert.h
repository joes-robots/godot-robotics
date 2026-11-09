#pragma once

#include <godot_cpp/variant/vector3.hpp>
#include <mujoco/mujoco.h>

// MuJoCo is Z-up, Godot is Y-up; both are right-handed.
// The mapping is a -90 degree rotation about X:
//   Godot  (x, y, z) -> MuJoCo (x, -z, y)
//   MuJoCo (x, y, z) -> Godot  (x, z, -y)
// All values crossing the Godot <-> MuJoCo boundary must go through these helpers.
namespace mujoco_convert {

inline godot::Vector3 to_godot(const mjtNum v[3]) {
	return godot::Vector3(
			static_cast<godot::real_t>(v[0]),
			static_cast<godot::real_t>(v[2]),
			static_cast<godot::real_t>(-v[1]));
}

inline void to_mujoco(const godot::Vector3 &v, mjtNum out[3]) {
	out[0] = v.x;
	out[1] = -v.z;
	out[2] = v.y;
}

} // namespace mujoco_convert
