#pragma once

#include <godot_cpp/variant/basis.hpp>
#include <godot_cpp/variant/quaternion.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <mujoco/mujoco.h>

// MuJoCo is Z-up, Godot is Y-up; both are right-handed.
// The mapping is a -90 degree rotation about X:
//   Godot  (x, y, z) -> MuJoCo (x, -z, y)
//   MuJoCo (x, y, z) -> Godot  (x, z, -y)
// All values crossing the Godot <-> MuJoCo boundary must go through these helpers.
//
// For a node's own axes this means: MuJoCo local x, y, z are Godot local
// x, -z, y. So a capsule/cylinder axis or plane normal (MuJoCo local z) is the
// node's local Y in Godot, matching Godot's CapsuleMesh, CylinderMesh and PlaneMesh.
namespace mujoco_convert {

// Change of basis C with v_mujoco = C * v_godot. Columns are where Godot's
// x, y and z axes land in MuJoCo coordinates.
inline godot::Basis godot_to_mujoco_basis() {
	return godot::Basis(
			godot::Vector3(1, 0, 0),
			godot::Vector3(0, 0, 1),
			godot::Vector3(0, -1, 0));
}

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

// Godot transform -> MuJoCo position and quaternion (w, x, y, z). Scale is
// dropped from the rotation (MuJoCo frames have no scale); the origin is used as is.
inline void to_mujoco(const godot::Transform3D &t, mjtNum pos[3], mjtNum quat[4]) {
	to_mujoco(t.origin, pos);
	const godot::Basis c = godot_to_mujoco_basis();
	const godot::Quaternion q = (c * t.basis * c.transposed()).get_rotation_quaternion();
	quat[0] = q.w;
	quat[1] = q.x;
	quat[2] = q.y;
	quat[3] = q.z;
}

// MuJoCo position and quaternion (w, x, y, z) -> Godot transform.
inline godot::Transform3D to_godot(const mjtNum pos[3], const mjtNum quat[4]) {
	const godot::Basis c = godot_to_mujoco_basis();
	const godot::Quaternion q(
			static_cast<godot::real_t>(quat[1]),
			static_cast<godot::real_t>(quat[2]),
			static_cast<godot::real_t>(quat[3]),
			static_cast<godot::real_t>(quat[0]));
	return godot::Transform3D(c.transposed() * godot::Basis(q.normalized()) * c, to_godot(pos));
}

} // namespace mujoco_convert
