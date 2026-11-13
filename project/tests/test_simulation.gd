extends "res://tests/test_base.gd"
## Stepping: bodies with joints move under MuJoCo, and their nodes follow.
## Run with --fixed-fps 60 (scripts/run_tests.sh does), so every physics frame
## is exactly 1/60 s.

const TIMESTEP := 0.002


func new_node(type: String, parent: Node, node_name: String) -> Node3D:
	return add(parent, ClassDB.instantiate(type), node_name)


## A body with a free joint and a 1 m cube, i.e. something that can fall.
func new_free_box(parent: Node, box_name: String, at: Vector3) -> Node3D:
	var body: Node3D = ClassDB.instantiate("MujocoBody")
	body.position = at
	add(parent, body, box_name)
	var joint := new_node("MujocoJoint", body, "Free")
	joint.type = ClassDB.class_get_integer_constant("MujocoJoint", "TYPE_FREE")
	var geom := new_node("MujocoGeom", body, "Geom")
	geom.shape = ClassDB.class_get_integer_constant("MujocoGeom", "SHAPE_BOX")
	return body


func run() -> void:
	check("MujocoJoint is registered", ClassDB.class_exists("MujocoJoint"))
	await test_free_fall()
	await test_automatic_stepping()
	await test_recompile_during_play()
	test_gravity_option()
	test_resting_on_floor()
	test_nested_poses()
	test_default_joint()


func test_free_fall() -> void:
	var world := new_node("MujocoWorld", root, "Fall")
	world.paused = true
	var box := new_free_box(world, "Box", Vector3(1, 10, 2))
	check("compiles", world.compile())
	check_near("time starts at zero", world.get_time(), 0.0, 1e-12)

	# Stepping by hand: 250 steps of 2 ms is half a second.
	world.step(250)
	check_near("time advances by steps x timestep", world.get_time(), 0.5, 1e-9)
	# Free fall from rest: y = y0 - g t^2 / 2. MuJoCo's integrator is a little
	# off the exact formula, by about g * t * timestep / 2 = 0.005 m here.
	check_near("the box falls along Godot's -y", box.position.y, 10.0 - 0.5 * G * 0.25, 0.01)
	check_near("and not sideways (x)", box.position.x, 1.0, 1e-5)
	check_near("and not sideways (z)", box.position.z, 2.0, 1e-5)

	# Paused: physics frames go by and nothing moves. (This also checks that the
	# compile() above met the recompile requests queued while the box was being
	# built: a second, queued compile would have restarted time.)
	var time_before: float = world.get_time()
	for i in 5:
		await physics_frame
	check_near("paused world doesn't advance", world.get_time(), time_before, 1e-12)

	# Reset: back to the start.
	world.reset()
	check_near("reset returns to time zero", world.get_time(), 0.0, 1e-12)
	check("reset returns the box to where it was placed", box.position.is_equal_approx(Vector3(1, 10, 2)))
	world.free()


func test_automatic_stepping() -> void:
	var world := new_node("MujocoWorld", root, "Auto")
	var box := new_free_box(world, "Box", Vector3(0, 10, 0))
	world.compile()
	# 30 physics frames at 60 per second is half a second.
	for i in 30:
		await physics_frame
	check_near("an unpaused world keeps up with game time", world.get_time(), 0.5, 2.0 / 60.0)
	check("whole timesteps only", is_equal_approx(fmod(world.get_time() + TIMESTEP / 2, TIMESTEP), TIMESTEP / 2))
	# Being moved by the simulation must not recompile: that would restart time.
	check("following the simulation doesn't recompile", world.get_time() > 0.4)
	check("the node has moved down", box.position.y < 9.5)
	world.free()


func test_recompile_during_play() -> void:
	var world := new_node("MujocoWorld", root, "Edit")
	world.paused = true
	var box := new_free_box(world, "Box", Vector3(0, 10, 0))
	world.compile()
	world.step(250)
	check("the box has fallen", box.position.y < 9.0)

	# A structural change rebuilds the model from where things were placed.
	new_free_box(world, "Other", Vector3(3, 1, 0))
	await process_frame
	check_near("recompiling restarts time", world.get_time(), 0.0, 1e-12)
	check("and returns the box to where it was placed", box.position.is_equal_approx(Vector3(0, 10, 0)))

	# Moving a body yourself places it: that is its new starting point.
	world.step(250)
	box.position = Vector3(0, 20, 0)
	await process_frame
	check("a moved body starts from its new position", box.position.is_equal_approx(Vector3(0, 20, 0)))
	world.step(250)
	world.reset()
	check("and reset returns to the new position", box.position.is_equal_approx(Vector3(0, 20, 0)))
	world.free()


func test_gravity_option() -> void:
	var world := new_node("MujocoWorld", root, "Gravity")
	world.paused = true
	var box := new_free_box(world, "Box", Vector3.ZERO)
	world.compile()
	# Changed after compiling: this goes straight into the compiled model.
	world.set("option_gravity", Vector3(2, 0, 0))
	world.step(250)
	check_near("gravity option acts along Godot's +x", box.position.x, 0.5 * 2.0 * 0.25, 0.005)
	check_near("with nothing along y", box.position.y, 0.0, 1e-5)
	check_near("or z", box.position.z, 0.0, 1e-5)

	world.reset()
	world.set("option_gravity", Vector3(0, 0, -2))
	world.step(250)
	check_near("gravity option acts along Godot's -z", box.position.z, -0.5 * 2.0 * 0.25, 0.005)
	world.free()


func test_resting_on_floor() -> void:
	var world := new_node("MujocoWorld", root, "Floor")
	world.paused = true
	var floor_geom := new_node("MujocoGeom", world, "Ground")
	floor_geom.shape = ClassDB.class_get_integer_constant("MujocoGeom", "SHAPE_PLANE")
	floor_geom.size = Vector3(0, 0.1, 0)  # infinite plane
	var box := new_free_box(world, "Box", Vector3(0, 2, 0))
	check("compiles", world.compile())
	world.step(1500)  # 3 seconds
	# The cube's half-height is 0.5, so resting on a floor at y = 0 its centre is at 0.5.
	check_near("the box comes to rest on the floor", box.position.y, 0.5, 0.01)
	check("and stays upright", box.basis.is_equal_approx(Basis.IDENTITY))
	world.free()


func test_nested_poses() -> void:
	# The world node is moved and turned; a body sits under a grouping node and
	# has a child body. Node poses must still come out right.
	var world := new_node("MujocoWorld", root, "Nested")
	world.paused = true
	world.position = Vector3(100, 0, 0)
	world.rotation = Vector3(0, PI / 2, 0)
	world.set("option_gravity", Vector3.ZERO)
	var group: Node3D = add(world, Node3D.new(), "Group")
	group.position = Vector3(0, 5, 0)
	var box := new_free_box(group, "Box", Vector3(1, 0, 0))
	var child := new_node("MujocoBody", box, "Child")
	child.position = Vector3(0, 1, 0)
	var box_global := box.global_transform
	var child_global := child.global_transform
	world.compile()
	world.step(10)
	check("a body under a group in a moved world stays put", box.global_transform.is_equal_approx(box_global))
	check("and so does its child body", child.global_transform.is_equal_approx(child_global))
	check("the child's local transform is unchanged", child.position.is_equal_approx(Vector3(0, 1, 0)))
	world.free()


func test_default_joint() -> void:
	# With no type set, a joint is MuJoCo's default: a hinge.
	var world := new_node("MujocoWorld", root, "Hinge")
	world.paused = true
	var body := new_node("MujocoBody", world, "Arm")
	var joint := new_node("MujocoJoint", body, "Pivot")
	new_node("MujocoGeom", body, "Geom").position = Vector3(1, 0, 0)
	check("default joint type is hinge", joint.type == ClassDB.class_get_integer_constant("MujocoJoint", "TYPE_HINGE"))
	check("a hinged body compiles", world.compile())
	check("the joint is in the model", mjcf_elements(world.to_mjcf()).has("Arm/Pivot"))
	world.free()
