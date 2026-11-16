extends "res://tests/test_base.gd"
## The runtime API: element ids, joint state on nodes, whole-state arrays on
## the world, and the world's signals.

var FREE: int = ClassDB.class_get_integer_constant("MujocoJoint", "TYPE_FREE")
var BALL: int = ClassDB.class_get_integer_constant("MujocoJoint", "TYPE_BALL")
var SLIDE: int = ClassDB.class_get_integer_constant("MujocoJoint", "TYPE_SLIDE")

var compiled_count := 0
var stepped_count := 0


func new_node(type: String, parent: Node, node_name: String) -> Node3D:
	return add(parent, ClassDB.instantiate(type), node_name)


## A body with one joint and a small 1 kg sphere at `geom_at`.
func new_jointed_body(parent: Node, body_name: String, geom_at := Vector3.ZERO) -> Array:
	var body := new_node("MujocoBody", parent, body_name)
	var joint := new_node("MujocoJoint", body, "Joint")
	var geom := new_node("MujocoGeom", body, "Geom")
	geom.radius = 0.1
	geom.mass = 1.0
	geom.position = geom_at
	return [body, joint, geom]


func property_info(node: Object, property_name: String) -> Dictionary:
	for property in node.get_property_list():
		if property.name == property_name:
			return property
	return {}


func new_world(world_name: String) -> Node3D:
	var world := new_node("MujocoWorld", root, world_name)
	world.paused = true
	# No gravity: nothing moves unless a test makes it.
	world.set("option_gravity", Vector3.ZERO)
	return world


func run() -> void:
	test_ids()
	test_joint_state()
	test_state_arrays()
	await test_signals()


func test_ids() -> void:
	var world := new_world("Ids")
	var first := new_jointed_body(world, "First")
	var second := new_jointed_body(world, "Second")
	check("no id before compiling", first[0].get_mujoco_id() == -1 and first[1].get_mujoco_id() == -1)
	world.compile()
	# Body 0 is MuJoCo's own world body, so the first body node is 1.
	check("bodies are numbered in tree order, after the world body", first[0].get_mujoco_id() == 1 and second[0].get_mujoco_id() == 2)
	check("joints are numbered from 0", first[1].get_mujoco_id() == 0 and second[1].get_mujoco_id() == 1)
	check("geoms are numbered from 0", first[2].get_mujoco_id() == 0 and second[2].get_mujoco_id() == 1)

	# An element that isn't in the model has no id.
	var stray := new_node("MujocoBody", first[2], "Stray")  # under a geom: ignored
	world.compile()
	check("an element left out of the model has no id", stray.get_mujoco_id() == -1)

	# Ids follow the model: remove the first body and the second moves up.
	world.remove_child(first[0])
	check("an element removed from the world loses its id", first[0].get_mujoco_id() == -1 and first[1].get_mujoco_id() == -1)
	world.compile()
	check("ids are reassigned on every compile", second[0].get_mujoco_id() == 1 and second[1].get_mujoco_id() == 0)
	first[0].free()
	world.free()


func test_joint_state() -> void:
	var world := new_world("State")
	# A hinge about Godot's z with its weight out along x.
	var hinged := new_jointed_body(world, "Hinged", Vector3(1, 0, 0))
	var hinge: Node3D = hinged[1]
	hinge.axis = Vector3(0, 0, 1)
	var sliding := new_jointed_body(world, "Sliding")
	var slide: Node3D = sliding[1]
	slide.type = SLIDE
	slide.axis = Vector3(1, 0, 0)

	check_near("no state before compiling", hinge.joint_position, 0.0, 1e-12)
	world.compile()
	check_near("position starts at zero", hinge.joint_position, 0.0, 1e-12)

	# Setting a position moves the body at once, with no step.
	hinge.joint_position = 0.5
	check_near("position reads back", hinge.joint_position, 0.5, 1e-12)
	check_near("the hinged body turns to match, about Godot's z", hinged[0].rotation.z, 0.5, 1e-5)
	slide.joint_position = 0.25
	check_near("the sliding body moves to match, along Godot's x", sliding[0].position.x, 0.25, 1e-6)
	check_near("setting one joint leaves the other alone", hinge.joint_position, 0.5, 1e-12)

	# A velocity carries the joint forward when the simulation steps.
	hinge.joint_velocity = 2.0
	slide.joint_velocity = -1.0
	check_near("velocity reads back", hinge.joint_velocity, 2.0, 1e-12)
	world.step(100)  # 0.2 seconds
	check_near("the hinge has turned by velocity x time", hinge.joint_position, 0.5 + 2.0 * 0.2, 1e-6)
	check_near("the slide has moved by velocity x time", slide.joint_position, 0.25 - 1.0 * 0.2, 1e-6)
	# After a step the nodes show the state just reached, not the one before it.
	check_near("the body's pose is in step with the joint", hinged[0].rotation.z, hinge.joint_position, 1e-5)
	check_near("for the slide too", sliding[0].position.x, slide.joint_position, 1e-6)

	world.reset()
	check_near("reset returns position to zero", hinge.joint_position, 0.0, 1e-12)
	check_near("and velocity", hinge.joint_velocity, 0.0, 1e-12)

	# State is shown in the inspector but never saved.
	check("joint_position is editable on a hinge in a running game",
			property_info(hinge, "joint_position").usage & PROPERTY_USAGE_READ_ONLY == 0)
	check("and shown as an angle", property_info(hinge, "joint_position").hint_string.contains("radians_as_degrees"))
	check("on a slide it is a plain length", not property_info(slide, "joint_position").hint_string.contains("radians_as_degrees"))
	check("it is not saved with the scene", property_info(hinge, "joint_position").usage & PROPERTY_USAGE_STORAGE == 0)

	# Ball and free joints have no single number.
	hinge.type = BALL
	world.compile()
	check("joint_position is greyed out on a ball joint", property_info(hinge, "joint_position").usage & PROPERTY_USAGE_READ_ONLY != 0)
	check("and on a ball joint's velocity", property_info(hinge, "joint_velocity").usage & PROPERTY_USAGE_READ_ONLY != 0)
	hinge.joint_position = 1.0  # prints an error and does nothing
	check_near("setting it on a ball joint is refused", hinge.joint_position, 0.0, 1e-12)
	world.free()


func test_state_arrays() -> void:
	var world := new_world("Arrays")
	# A free body placed at Godot (1, 2, 3), then a hinge.
	var floating := new_jointed_body(world, "Floating")
	floating[0].position = Vector3(1, 2, 3)
	floating[1].type = FREE
	var hinged := new_jointed_body(world, "Hinged", Vector3(1, 0, 0))
	var hinge: Node3D = hinged[1]
	hinge.axis = Vector3(0, 0, 1)

	check("arrays are empty before compiling", world.get_qpos().is_empty() and world.get_qvel().is_empty())
	check("no addresses before compiling", hinge.get_qpos_address() == -1 and hinge.get_qvel_address() == -1)
	world.compile()

	# A free joint takes 7 positions (3 for where, 4 for which way round) and 6
	# velocities; a hinge takes one of each.
	var qpos: PackedFloat64Array = world.get_qpos()
	var qvel: PackedFloat64Array = world.get_qvel()
	check("qpos has an entry per joint coordinate", qpos.size() == 8)
	check("qvel has an entry per degree of freedom", qvel.size() == 7)
	check("the free joint comes first", floating[1].get_qpos_address() == 0 and floating[1].get_qvel_address() == 0)
	check("the hinge follows it", hinge.get_qpos_address() == 7 and hinge.get_qvel_address() == 6)

	# The arrays are MuJoCo's own numbers: Godot (1, 2, 3) is MuJoCo (1, -3, 2),
	# and an unrotated body's quaternion is (1, 0, 0, 0), w first.
	check("a free joint's qpos is in MuJoCo's convention",
			is_equal_approx(qpos[0], 1) and is_equal_approx(qpos[1], -3) and is_equal_approx(qpos[2], 2)
			and is_equal_approx(qpos[3], 1) and is_equal_approx(qpos[4], 0))

	# Writing the array moves everything at once.
	qpos[2] = 5.0  # MuJoCo z, which is Godot y
	qpos[7] = 0.3
	world.set_qpos(qpos)
	check_near("the free body moved up", floating[0].position.y, 5.0, 1e-6)
	check_near("the hinge turned", hinge.joint_position, 0.3, 1e-12)
	check_near("and its body with it", hinged[0].rotation.z, 0.3, 1e-5)

	qvel[6] = 1.5
	world.set_qvel(qvel)
	check_near("set_qvel reaches the joint", hinge.joint_velocity, 1.5, 1e-12)

	# An array of the wrong size is refused (prints an error).
	world.set_qpos(PackedFloat64Array([1.0, 2.0]))
	check_near("a wrong-sized qpos changes nothing", hinge.joint_position, 0.3, 1e-12)

	# Saving and restoring a state.
	var saved_qpos: PackedFloat64Array = world.get_qpos()
	var saved_qvel: PackedFloat64Array = world.get_qvel()
	world.step(200)
	check("the state has moved on", not is_equal_approx(hinge.joint_position, 0.3))
	world.set_qpos(saved_qpos)
	world.set_qvel(saved_qvel)
	check_near("restoring qpos brings it back", hinge.joint_position, 0.3, 1e-12)
	check_near("and qvel", hinge.joint_velocity, 1.5, 1e-12)
	world.free()


func test_signals() -> void:
	var world := new_world("Signals")
	world.connect("compiled", func(): compiled_count += 1)
	world.connect("stepped", func(): stepped_count += 1)
	var parts := new_jointed_body(world, "Body")

	compiled_count = 0
	check("compiled is emitted by a successful compile", world.compile() and compiled_count == 1)
	parts[2].radius = 0
	check("but not by a failed one", not world.compile() and compiled_count == 1)
	parts[2].radius = 0.1
	await process_frame
	check("and by an automatic recompile", compiled_count == 2)

	stepped_count = 0
	world.step(10)
	check("stepped is emitted once per step() call, not per timestep", stepped_count == 1)
	world.paused = false
	for i in 5:
		await physics_frame
	check("and once per physics frame while running", stepped_count >= 5 and stepped_count <= 7)
	world.paused = true
	world.free()
