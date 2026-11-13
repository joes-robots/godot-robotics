extends "res://tests/test_base.gd"
## Live attributes: some attribute changes go straight into the running model
## instead of rebuilding it. "Not rebuilt" is checked by simulated time carrying
## on (a rebuild restarts it at zero) and by the compiled signal not firing.

var FREE: int = ClassDB.class_get_integer_constant("MujocoJoint", "TYPE_FREE")
var SLIDE: int = ClassDB.class_get_integer_constant("MujocoJoint", "TYPE_SLIDE")
var BOX: int = ClassDB.class_get_integer_constant("MujocoGeom", "SHAPE_BOX")
var PLANE: int = ClassDB.class_get_integer_constant("MujocoGeom", "SHAPE_PLANE")

var compiles := 0


func new_node(type: String, parent: Node, node_name: String) -> Node3D:
	return add(parent, ClassDB.instantiate(type), node_name)


func new_world(world_name: String, gravity := Vector3.ZERO) -> Node3D:
	var world := new_node("MujocoWorld", root, world_name)
	world.paused = true
	world.set("option_gravity", gravity)
	world.connect("compiled", func(): compiles += 1)
	return world


## A world with one body on a joint, carrying a 1 kg sphere at `geom_at`.
func new_jointed_world(world_name: String, gravity := Vector3.ZERO, geom_at := Vector3.ZERO) -> Array:
	var world := new_world(world_name, gravity)
	var body := new_node("MujocoBody", world, "Body")
	var joint := new_node("MujocoJoint", body, "Joint")
	var geom := new_node("MujocoGeom", body, "Geom")
	geom.radius = 0.1
	geom.mass = 1.0
	geom.position = geom_at
	world.compile()
	return [world, body, joint, geom]


## Lets any queued recompile happen, then reports whether one did.
func rebuilt_since(count_before: int) -> bool:
	await process_frame
	return compiles != count_before


func run() -> void:
	await test_damping()
	await test_stiffness()
	await test_limits()
	await test_frictionloss()
	await test_armature()
	await test_geom_friction()
	await test_color()
	await test_others_still_rebuild()


func test_damping() -> void:
	var parts := new_jointed_world("Damping")
	var world: Node3D = parts[0]
	var joint: Node3D = parts[2]
	joint.type = SLIDE
	world.compile()

	# No damping: a moving joint keeps its speed.
	joint.joint_velocity = 1.0
	world.step(100)
	check_near("undamped, the joint keeps its speed", joint.joint_velocity, 1.0, 1e-6)

	# Turn damping on while it runs. With 1 kg and damping 5, speed falls as
	# exp(-5 t): to about 0.37 of itself in 0.2 seconds.
	var before := compiles
	var time_before: float = world.get_time()
	joint.damping = 5.0
	check("changing damping doesn't rebuild the model", not await rebuilt_since(before))
	check_near("or restart the simulation", world.get_time(), time_before, 1e-12)
	world.step(100)
	check_near("and takes effect", joint.joint_velocity, exp(-1.0), 0.01)

	# Removing the override is live too: back to no damping.
	before = compiles
	joint.damping = null
	check("reverting damping doesn't rebuild either", not await rebuilt_since(before))
	var speed: float = joint.joint_velocity
	world.step(100)
	check_near("and the joint coasts again", joint.joint_velocity, speed, 1e-6)

	# The change is in the node tree as well, so a later rebuild keeps it.
	joint.damping = 5.0
	world.compile()
	check_numbers("a later rebuild has the live value", str(mjcf_elements(world.to_mjcf())["Body/Joint"].get("damping", "0")).split(" ")[0], "5")
	world.free()


func test_stiffness() -> void:
	var parts := new_jointed_world("Stiffness", Vector3(0, -G, 0))
	var world: Node3D = parts[0]
	var body: Node3D = parts[1]
	var joint: Node3D = parts[2]
	joint.type = SLIDE
	joint.damping = 20
	joint.stiffness = 100
	world.compile()
	world.step(2500)
	check_near("settles on a spring of stiffness 100", body.position.y, -G / 100.0, 0.001)

	var before := compiles
	joint.stiffness = 200
	check("changing stiffness doesn't rebuild the model", not await rebuilt_since(before))
	world.step(2500)
	check_near("and the weight settles higher on the stiffer spring", body.position.y, -G / 200.0, 0.001)
	world.free()


func test_limits() -> void:
	# A weighted arm on a horizontal hinge, swinging down under gravity.
	var parts := new_jointed_world("Limits", Vector3(0, -G, 0), Vector3(1, 0, 0))
	var world: Node3D = parts[0]
	var joint: Node3D = parts[2]
	joint.axis = Vector3(0, 0, 1)
	joint.damping = 5
	world.compile()

	var before := compiles
	joint.range_lower = -0.5
	joint.range_upper = 0.5
	joint.limited = true
	check("setting limits doesn't rebuild the model", not await rebuilt_since(before))
	world.step(2500)
	check_near("and the arm stops at the new limit", joint.joint_position, -0.5, 0.02)

	before = compiles
	joint.range_lower = -1.0
	check("moving a limit doesn't rebuild", not await rebuilt_since(before))
	world.step(2500)
	check_near("the arm drops to the moved limit", joint.joint_position, -1.0, 0.02)

	before = compiles
	joint.limited = false
	check("removing the limit doesn't rebuild", not await rebuilt_since(before))
	world.step(10000)
	check_near("and the arm hangs straight down", joint.joint_position, -PI / 2, 0.02)
	world.free()


func test_frictionloss() -> void:
	var parts := new_jointed_world("FrictionLoss")
	var world: Node3D = parts[0]
	var joint: Node3D = parts[2]
	joint.type = SLIDE
	world.compile()
	joint.joint_velocity = 1.0

	# Dry friction of 2 N on 1 kg takes 2 m/s off every second: a joint moving
	# at 1 m/s stops within half a second and stays stopped.
	var before := compiles
	joint.frictionloss = 2.0
	check("changing frictionloss doesn't rebuild the model", not await rebuilt_since(before))
	world.step(150)  # 0.3 seconds
	check_near("friction slows the joint steadily", joint.joint_velocity, 1.0 - 2.0 * 0.3, 0.02)
	world.step(500)
	check_near("and stops it", joint.joint_velocity, 0.0, 1e-3)
	world.free()


func test_armature() -> void:
	var parts := new_jointed_world("Armature")
	var world: Node3D = parts[0]
	var joint: Node3D = parts[2]
	joint.type = SLIDE
	world.compile()

	# 3 N on 1 kg: 3 m/s^2.
	joint.applied_force = 3.0
	world.step(250)
	check_near("without armature", joint.joint_velocity, 1.5, 1e-6)

	# Armature adds inertia: with 2 more, the same force gives 1 m/s^2.
	var before := compiles
	var position_before: float = joint.joint_position
	joint.armature = 2.0
	check("changing armature doesn't rebuild the model", not await rebuilt_since(before))
	check_near("or disturb the joint's position", joint.joint_position, position_before, 1e-12)
	check_near("or its velocity", joint.joint_velocity, 1.5, 1e-12)
	world.step(250)
	check_near("and the joint now accelerates a third as fast", joint.joint_velocity, 1.5 + 0.5, 1e-5)
	world.free()


func test_geom_friction() -> void:
	# A box sliding across a floor.
	var world := new_world("GeomFriction", Vector3(0, -G, 0))
	var floor_geom := new_node("MujocoGeom", world, "Floor")
	floor_geom.shape = PLANE
	floor_geom.friction_sliding = 0.0
	var body := new_node("MujocoBody", world, "Box")
	body.position = Vector3(0, 0.5, 0)
	var joint := new_node("MujocoJoint", body, "Free")
	joint.type = FREE
	var box := new_node("MujocoGeom", body, "Geom")
	box.shape = BOX
	box.friction_sliding = 0.0
	world.compile()
	world.step(250)  # let it settle on the floor

	# Push it along x. MuJoCo uses the larger of the two geoms' friction, so
	# with both at zero it slides freely.
	var qvel: PackedFloat64Array = world.get_qvel()
	qvel[0] = 2.0
	world.set_qvel(qvel)
	world.step(250)
	check_near("with no friction the box keeps sliding", body.get_linear_velocity().x, 2.0, 0.01)

	var before := compiles
	floor_geom.friction_sliding = 1.0
	check("changing geom friction doesn't rebuild the model", not await rebuilt_since(before))
	world.step(250)
	check("and the box is brought to a stop", absf(body.get_linear_velocity().x) < 0.01)
	world.free()


func test_color() -> void:
	var parts := new_jointed_world("Color")
	var world: Node3D = parts[0]
	var geom: Node3D = parts[3]
	world.step(50)
	var before := compiles
	geom.color = Color(1, 0, 0, 1)
	check("changing color doesn't rebuild the model", not await rebuilt_since(before))
	check_near("or restart the simulation", world.get_time(), 0.1, 1e-9)
	check("the drawn shape has the new colour", (geom.get_child(0, true) as MeshInstance3D).material_override.albedo_color == Color(1, 0, 0, 1))
	world.free()


func test_others_still_rebuild() -> void:
	var parts := new_jointed_world("Rebuild")
	var world: Node3D = parts[0]
	var joint: Node3D = parts[2]
	var geom: Node3D = parts[3]
	world.step(50)

	var before := compiles
	geom.radius = 0.2
	check("changing a geom's size rebuilds the model", await rebuilt_since(before))
	check_near("which restarts the simulation", world.get_time(), 0.0, 1e-12)

	before = compiles
	joint.axis = Vector3(1, 0, 0)
	check("changing a joint's axis rebuilds the model", await rebuilt_since(before))

	before = compiles
	geom.mass = 2.0
	check("changing a geom's mass rebuilds the model", await rebuilt_since(before))
	world.free()
