extends "res://tests/test_base.gd"
## MujocoBody at run time: mocap bodies, applied forces and torques, velocities.

var FREE: int = ClassDB.class_get_integer_constant("MujocoJoint", "TYPE_FREE")
var SLIDE: int = ClassDB.class_get_integer_constant("MujocoJoint", "TYPE_SLIDE")
var BOX: int = ClassDB.class_get_integer_constant("MujocoGeom", "SHAPE_BOX")


func new_node(type: String, parent: Node, node_name: String) -> Node3D:
	return add(parent, ClassDB.instantiate(type), node_name)


func new_world(world_name: String) -> Node3D:
	var world := new_node("MujocoWorld", root, world_name)
	world.paused = true
	world.set("option_gravity", Vector3.ZERO)
	return world


## A free body carrying a 1 kg sphere of radius 0.1.
func new_free_ball(world: Node, ball_name: String, at: Vector3) -> Node3D:
	var body := new_node("MujocoBody", world, ball_name)
	body.position = at
	new_node("MujocoJoint", body, "Free").type = FREE
	var geom := new_node("MujocoGeom", body, "Geom")
	geom.radius = 0.1
	geom.mass = 1.0
	return body


func inspector_section(node: Object, property_name: String) -> String:
	var section := ""
	for property in node.get_property_list():
		if property.usage & PROPERTY_USAGE_CATEGORY:
			section = property.name
		elif property.name == property_name:
			return section
	return ""


func run() -> void:
	await test_mocap()
	test_mocap_in_moved_world()
	test_forces()
	test_forces_in_turned_world()
	test_joint_force()


func test_mocap() -> void:
	var world := new_world("Mocap")
	# A mocap paddle, and a free ball a little way along +x from it.
	var paddle := new_node("MujocoBody", world, "Paddle")
	var paddle_geom := new_node("MujocoGeom", paddle, "Geom")
	paddle_geom.shape = BOX
	paddle_geom.half_extents = Vector3(0.1, 0.5, 0.5)
	var ball := new_free_ball(world, "Ball", Vector3(1, 0, 0))

	check("mocap is off by default", paddle.mocap == false)
	check("mocap is listed under MujocoBody", inspector_section(paddle, "mocap") == "MujocoBody")
	paddle.mocap = true
	check("compiles", world.compile())
	check("the model has a mocap body", mjcf_elements(world.to_mjcf())["Paddle"].get("mocap", "") == "true")

	# Moving a mocap body in a running game moves it in the simulation: no rebuild.
	world.step(50)
	var time_before: float = world.get_time()
	paddle.position = Vector3(0.5, 0, 0)
	await process_frame
	check_near("moving a mocap body doesn't restart the simulation", world.get_time(), time_before, 1e-12)
	world.step(1)
	# If the move hadn't reached the simulation, this step would have put the node back.
	check("the simulation has the new pose", paddle.position.is_equal_approx(Vector3(0.5, 0, 0)))

	# It pushes other bodies: slide it into the ball.
	for i in 100:
		paddle.position = Vector3(0.5 + 0.005 * (i + 1), 0, 0)
		world.step(5)
	check("a mocap body pushes what it meets", ball.position.x > 1.1)
	check_near("without being pushed back itself", paddle.position.x, 1.0, 1e-6)

	# Rotation goes through as well.
	paddle.rotation = Vector3(0, 0, 0.3)
	world.step(1)
	check_near("a mocap body can be turned", paddle.rotation.z, 0.3, 1e-5)

	# Its rest pose is where it was placed: reset puts it back.
	world.reset()
	check("reset returns a mocap body to where it was placed", paddle.position.is_equal_approx(Vector3.ZERO) and paddle.rotation.is_equal_approx(Vector3.ZERO))

	# A body that isn't mocap is placed, not moved: the model is rebuilt.
	world.step(50)
	paddle.mocap = false
	await process_frame
	world.step(50)
	paddle.position = Vector3(0, 1, 0)
	await process_frame
	check_near("moving an ordinary body restarts the simulation", world.get_time(), 0.0, 1e-12)
	world.free()


func test_mocap_in_moved_world() -> void:
	# The simulation's frame is the world node's, wherever that node is.
	var world := new_world("MocapMoved")
	world.position = Vector3(10, 0, 0)
	world.rotation = Vector3(0, PI / 2, 0)
	var paddle := new_node("MujocoBody", world, "Paddle")
	new_node("MujocoGeom", paddle, "Geom")
	paddle.mocap = true
	world.compile()
	paddle.global_position = Vector3(10, 2, 3)
	var placed := paddle.global_transform
	world.step(5)
	check("a mocap body in a moved, turned world stays where it is put", paddle.global_transform.is_equal_approx(placed))
	world.free()


func test_forces() -> void:
	var world := new_world("Forces")
	var ball := new_free_ball(world, "Ball", Vector3.ZERO)
	check("no force before compiling", ball.applied_force == Vector3.ZERO)
	world.compile()

	# 2 N on 1 kg for half a second: v = a t = 1 m/s, x = a t^2 / 2 = 0.25 m.
	ball.applied_force = Vector3(2, 0, 0)
	check("the force reads back", ball.applied_force.is_equal_approx(Vector3(2, 0, 0)))
	world.step(250)
	check_near("a force accelerates the body", ball.position.x, 0.25, 0.003)
	check("along the right axis", absf(ball.position.y) < 1e-6 and absf(ball.position.z) < 1e-6)
	check("linear velocity", ball.get_linear_velocity().is_equal_approx(Vector3(1, 0, 0)))

	# The force stays until changed; without it the body coasts.
	ball.applied_force = Vector3.ZERO
	var x_before: float = ball.position.x
	world.step(250)
	check_near("with the force removed the body coasts", ball.position.x - x_before, 0.5, 0.003)

	# Each axis goes the right way.
	world.reset()
	check("reset clears applied forces", ball.applied_force == Vector3.ZERO)
	ball.applied_force = Vector3(0, 2, -4)
	world.step(250)
	check("force along Godot's y and z", is_equal_approx(ball.position.x, 0.0)
			and absf(ball.position.y - 0.25) < 0.003 and absf(ball.position.z + 0.5) < 0.006)

	# Torque: a solid sphere's moment of inertia is 0.4 m r^2 = 0.004, so a
	# torque of 0.004 N m spins it up at 1 rad/s^2.
	world.reset()
	ball.applied_torque = Vector3(0, 0.004, 0)
	check("the torque reads back", ball.applied_torque.is_equal_approx(Vector3(0, 0.004, 0)))
	world.step(250)
	check("a torque spins the body up, about the right axis", ball.get_angular_velocity().is_equal_approx(Vector3(0, 0.5, 0)))
	check("without moving it", ball.position.length() < 1e-6)

	check("applied forces are not saved", not _is_stored(ball, "applied_force") and not _is_stored(ball, "applied_torque"))
	world.free()


func _is_stored(node: Object, property_name: String) -> bool:
	for property in node.get_property_list():
		if property.name == property_name:
			return property.usage & PROPERTY_USAGE_STORAGE != 0
	return false


func test_forces_in_turned_world() -> void:
	# Forces and velocities are in global coordinates, however the world node is turned.
	var world := new_world("ForcesTurned")
	world.rotation = Vector3(0, PI / 2, 0)
	var ball := new_free_ball(world, "Ball", Vector3.ZERO)
	world.compile()
	ball.applied_force = Vector3(2, 0, 0)
	check("the force reads back in global coordinates", ball.applied_force.is_equal_approx(Vector3(2, 0, 0)))
	world.step(250)
	check_near("a global force moves the body along the global axis", ball.global_position.x, 0.25, 0.003)
	check("and nowhere else", absf(ball.global_position.y) < 1e-5 and absf(ball.global_position.z) < 1e-5)
	check("velocity is global too", ball.get_linear_velocity().is_equal_approx(Vector3(1, 0, 0)))
	world.free()


func test_joint_force() -> void:
	var world := new_world("JointForce")
	var body := new_node("MujocoBody", world, "Slider")
	var joint := new_node("MujocoJoint", body, "Joint")
	joint.type = SLIDE
	joint.axis = Vector3(1, 0, 0)
	var geom := new_node("MujocoGeom", body, "Geom")
	geom.radius = 0.1
	geom.mass = 1.0
	world.compile()

	# 3 N along a slide carrying 1 kg: 3 m/s^2.
	joint.applied_force = 3.0
	check_near("the joint force reads back", joint.applied_force, 3.0, 1e-12)
	world.step(250)
	check_near("a joint force accelerates the joint", joint.joint_velocity, 1.5, 1e-6)
	world.reset()
	check_near("reset clears it", joint.applied_force, 0.0, 1e-12)

	joint.type = FREE
	world.compile()
	joint.applied_force = 1.0  # prints an error and does nothing
	check_near("it is refused on a free joint", joint.applied_force, 0.0, 1e-12)
	world.free()
