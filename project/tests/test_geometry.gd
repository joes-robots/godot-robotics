extends "res://tests/test_base.gd"
## Godot (Y-up) -> MuJoCo (Z-up) conversion of positions, rotations and geom sizes,
## checked against hand-derived values in the exported MJCF.


func run() -> void:
	var world := make_world("World")

	var floor_geom: MujocoGeom = add(world, MujocoGeom.new(), "Floor")
	floor_geom.shape = MujocoGeom.SHAPE_PLANE
	floor_geom.size = Vector3(20, 0, 20)

	var offset: Node3D = add(world, Node3D.new(), "Offset")
	offset.position = Vector3(0, 1, 0)

	var box: MujocoBody = add(offset, MujocoBody.new(), "Box")
	box.position = Vector3(1, 2, 3)
	box.rotation_degrees = Vector3(0, 90, 0)

	var box_geom: MujocoGeom = add(box, MujocoGeom.new(), "BoxGeom")
	box_geom.size = Vector3(2, 4, 6)

	var ball: MujocoGeom = add(box, MujocoGeom.new(), "Ball")
	ball.shape = MujocoGeom.SHAPE_SPHERE
	ball.radius = 0.25
	ball.position = Vector3(0, 0, -1)

	var arm: MujocoBody = add(box, MujocoBody.new(), "Arm")
	arm.rotation_degrees = Vector3(90, 0, 0)

	var arm_geom: MujocoGeom = add(arm, MujocoGeom.new(), "ArmGeom")
	arm_geom.shape = MujocoGeom.SHAPE_CAPSULE
	arm_geom.radius = 0.1
	arm_geom.height = 1.0
	arm_geom.rotation_degrees = Vector3(0, 0, 90)

	var hinge: MujocoJoint = add(arm, MujocoJoint.new(), "Hinge")
	hinge.position = Vector3(0, 0.5, 0)
	hinge.rotation_degrees = Vector3(0, 0, 90) # local Y -> Godot -X

	await process_frame
	var e := mjcf_elements(world.get_mjcf())

	check_numbers("plane: Godot 20x20 -> MuJoCo half-size", e["Floor"].get("size", ""), "10 10 1")
	check_numbers("body pos through a plain Node3D offset", e["Offset/Box"].get("pos", ""), "1 -3 3")
	check_numbers("90 deg about Godot Y = MuJoCo +Z", e["Offset/Box"].get("quat", ""), "0.707107 0 0 0.707107")
	check_numbers("box: Godot 2,4,6 -> MuJoCo half x,z,y", e["Offset/Box/BoxGeom"].get("size", ""), "1 3 2")
	check_numbers("geom pos: Godot local 0,0,-1", e["Offset/Box/Ball"].get("pos", ""), "0 1 0")
	check_numbers("sphere radius", e["Offset/Box/Ball"].get("size", ""), "0.25")
	check_numbers("90 deg about Godot X = MuJoCo +X", e["Offset/Box/Arm"].get("quat", ""), "0.707107 0.707107 0 0")
	check_numbers("90 deg about Godot Z = MuJoCo -Y", e["Offset/Box/Arm/ArmGeom"].get("quat", ""), "0.707107 0 -0.707107 0")
	check_numbers("capsule r=0.1 h=1.0 -> radius, half-length", e["Offset/Box/Arm/ArmGeom"].get("size", ""), "0.1 0.4")
	# Hinge at Godot (0, 0.5, 0) in the arm's frame = MuJoCo (0, 0, 0.5); its local Y is Godot -X = MuJoCo -X.
	check_numbers("joint anchor", e["Offset/Box/Arm/Hinge"].get("pos", ""), "0 0 0.5")
	check_numbers("joint axis is the node's local Y", e["Offset/Box/Arm/Hinge"].get("axis", ""), "-1 0 0")

	# Edits and removals are reflected after the next compile.
	ball.radius = 0.3
	arm.free()
	await process_frame
	e = mjcf_elements(world.get_mjcf())
	check_numbers("edited sphere radius", e["Offset/Box/Ball"].get("size", ""), "0.3")
	check("removed body is gone", not e.has("Offset/Box/Arm"))

	# The drawn ellipsoid is stretched in the mesh itself, not with a node scale.
	var ellipsoid: MujocoGeom = add(world, MujocoGeom.new(), "Ellipsoid")
	ellipsoid.shape = MujocoGeom.SHAPE_ELLIPSOID
	ellipsoid.size = Vector3(1, 2, 3)
	var shape_instance: MeshInstance3D = ellipsoid.get_child(0, true)
	# The sphere's faces sit slightly inside the true surface, hence the tolerance.
	check("ellipsoid mesh has the extents", shape_instance.mesh.get_aabb().size.distance_to(Vector3(1, 2, 3)) < 0.01)
	check("ellipsoid mesh is not scaled", shape_instance.scale == Vector3.ONE)

	# A geom outside any world is harmless.
	add(root, MujocoGeom.new(), "Stray")
	await process_frame
	check("geom outside a world", true)
