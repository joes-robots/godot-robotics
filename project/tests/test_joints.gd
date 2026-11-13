extends "res://tests/test_base.gd"
## MujocoJoint: attributes, axis handling, angle units, limits and springs,
## and which attributes the inspector greys out for each type.

var FREE: int = ClassDB.class_get_integer_constant("MujocoJoint", "TYPE_FREE")
var BALL: int = ClassDB.class_get_integer_constant("MujocoJoint", "TYPE_BALL")
var SLIDE: int = ClassDB.class_get_integer_constant("MujocoJoint", "TYPE_SLIDE")
var HINGE: int = ClassDB.class_get_integer_constant("MujocoJoint", "TYPE_HINGE")


func new_node(type: String, parent: Node, node_name: String) -> Node3D:
	return add(parent, ClassDB.instantiate(type), node_name)


## A world holding one body with one joint and a 1 kg sphere at `geom_at`.
func new_jointed_body(world_name: String, geom_at := Vector3.ZERO) -> Array:
	var world := new_node("MujocoWorld", root, world_name)
	world.paused = true
	var body := new_node("MujocoBody", world, "Body")
	var joint := new_node("MujocoJoint", body, "Joint")
	var geom := new_node("MujocoGeom", body, "Geom")
	geom.radius = 0.1
	geom.mass = 1.0
	geom.position = geom_at
	return [world, body, joint]


## The compiled model's attributes for the joint.
func compiled_joint(world: Node3D) -> Dictionary:
	check("compiles", world.compile())
	return mjcf_elements(world.to_mjcf()).get("Body/Joint", {})


## The entry for `property_name` in the node's property list.
func property_info(node: Object, property_name: String) -> Dictionary:
	for property in node.get_property_list():
		if property.name == property_name:
			return property
	return {}


func is_greyed_out(node: Object, property_name: String) -> bool:
	return property_info(node, property_name).usage & PROPERTY_USAGE_READ_ONLY != 0


func run() -> void:
	test_defaults()
	test_axis()
	test_attributes_reach_the_model()
	test_greying_out()
	test_hinge_limit()
	test_slide_spring()


func test_defaults() -> void:
	var parts := new_jointed_body("Defaults")
	var joint: Node3D = parts[2]
	check("default type is hinge", joint.type == HINGE)
	# MuJoCo's default axis is its z, which is up: Godot's y.
	check("default axis is Godot's up", joint.axis.is_equal_approx(Vector3(0, 1, 0)))
	check("not limited by default", joint.limited == false and joint.range_lower == 0.0 and joint.range_upper == 0.0)
	check("other defaults are zero", joint.ref == 0.0 and joint.springref == 0.0 and joint.stiffness == 0.0
			and joint.damping == 0.0 and joint.armature == 0.0 and joint.frictionloss == 0.0)
	check("nothing is stored", joint.get_overrides().is_empty())
	parts[0].free()


func test_axis() -> void:
	var parts := new_jointed_body("Axis")
	var world: Node3D = parts[0]
	var joint: Node3D = parts[2]

	# Godot x is MuJoCo x.
	joint.axis = Vector3(1, 0, 0)
	check_numbers("axis along x", compiled_joint(world).get("axis", ""), "1 0 0")
	# Godot z is MuJoCo -y.
	joint.axis = Vector3(0, 0, 1)
	check_numbers("axis along Godot z", compiled_joint(world).get("axis", ""), "0 -1 0")

	# The axis is in the joint node's own frame: turning the node turns it.
	# A quarter turn about x takes the default axis (y) to z.
	joint.axis = null
	joint.rotation = Vector3(PI / 2, 0, 0)
	check_numbers("rotating the node rotates the default axis", compiled_joint(world).get("axis", ""), "0 -1 0")
	joint.axis = Vector3(1, 0, 0)
	check_numbers("an axis along the turn axis is unaffected", compiled_joint(world).get("axis", ""), "1 0 0")

	# The node's position is the anchor. Godot (0, 0.5, -0.25) is MuJoCo (0, 0.25, 0.5).
	joint.position = Vector3(0, 0.5, -0.25)
	check_numbers("node position is the anchor", compiled_joint(world).get("pos", ""), "0 0.25 0.5")
	world.free()


func test_attributes_reach_the_model() -> void:
	var parts := new_jointed_body("Attributes")
	var world: Node3D = parts[0]
	var joint: Node3D = parts[2]
	joint.limited = true
	joint.range_lower = -PI / 2
	joint.range_upper = PI / 4
	joint.ref = 0.1
	joint.springref = 0.2
	joint.stiffness = 30
	joint.damping = 4
	joint.armature = 0.05
	joint.frictionloss = 0.6
	var attributes := compiled_joint(world)
	check("the model's angles are in radians", world.to_mjcf().contains("angle=\"radian\""))
	check_numbers("range, in radians", attributes.get("range", ""), "-1.5708 0.7854", 1e-4)
	check_numbers("ref", attributes.get("ref", ""), "0.1")
	check_numbers("springref", attributes.get("springref", ""), "0.2")
	check_near("stiffness", attributes.get("stiffness", "0").split_floats(" ")[0], 30.0, 1e-9)
	check_near("damping", attributes.get("damping", "0").split_floats(" ")[0], 4.0, 1e-9)
	check_numbers("armature", attributes.get("armature", ""), "0.05")
	check_numbers("frictionloss", attributes.get("frictionloss", ""), "0.6")
	world.free()


func test_greying_out() -> void:
	var parts := new_jointed_body("Grey")
	var world: Node3D = parts[0]
	var joint: Node3D = parts[2]

	check("unlimited hinge: the range is greyed out", is_greyed_out(joint, "range_lower") and is_greyed_out(joint, "range_upper"))
	joint.limited = true
	check("hinge: nothing greyed out", not is_greyed_out(joint, "axis") and not is_greyed_out(joint, "range_lower") and not is_greyed_out(joint, "range_upper")
			and not is_greyed_out(joint, "ref") and not is_greyed_out(joint, "damping"))
	check("hinge: range is shown in degrees", property_info(joint, "range_lower").hint_string.contains("radians_as_degrees")
			and property_info(joint, "range_upper").hint_string.contains("radians_as_degrees"))

	joint.type = SLIDE
	check("slide: nothing greyed out", not is_greyed_out(joint, "axis") and not is_greyed_out(joint, "range_lower") and not is_greyed_out(joint, "range_upper"))
	check("slide: range is a length, not shown in degrees", not property_info(joint, "range_upper").hint_string.contains("radians_as_degrees"))

	joint.type = BALL
	check("ball: axis, ref and springref are greyed out", is_greyed_out(joint, "axis") and is_greyed_out(joint, "ref") and is_greyed_out(joint, "springref"))
	check("ball: the lower limit is greyed out too (it is always 0)", is_greyed_out(joint, "range_lower"))
	check("ball: the upper limit and damping are not", not is_greyed_out(joint, "range_upper") and not is_greyed_out(joint, "damping"))

	joint.type = FREE
	check("free: axis, limits, ref and springref are greyed out", is_greyed_out(joint, "axis") and is_greyed_out(joint, "limited")
			and is_greyed_out(joint, "range_lower") and is_greyed_out(joint, "range_upper")
			and is_greyed_out(joint, "ref") and is_greyed_out(joint, "springref"))
	check("free: type and damping are not", not is_greyed_out(joint, "type") and not is_greyed_out(joint, "damping"))

	# A greyed-out attribute keeps its value but is left out of the model.
	joint.axis = Vector3(1, 0, 0)
	joint.range_lower = -1
	joint.range_upper = 1
	check("greyed-out values are kept", joint.has_override("axis") and joint.has_override("range_upper"))
	var attributes := compiled_joint(world)
	check("but not written to the model", not attributes.has("axis") and not attributes.has("range"))
	joint.type = HINGE
	check("and apply again when the type is switched back", compiled_joint(world).has("range"))
	world.free()


func test_hinge_limit() -> void:
	# A weight on a horizontal arm, hinged about Godot's z. Gravity swings it
	# down, clockwise seen from +z, which is a negative angle about z.
	var parts := new_jointed_body("Limit", Vector3(1, 0, 0))
	var world: Node3D = parts[0]
	var body: Node3D = parts[1]
	var joint: Node3D = parts[2]
	joint.axis = Vector3(0, 0, 1)
	joint.damping = 5
	joint.limited = true
	joint.range_lower = -0.5
	joint.range_upper = 0.5
	check("compiles", world.compile())
	world.step(2500)  # 5 seconds
	check_near("the arm swings down and stops at its lower limit", body.rotation.z, -0.5, 0.02)

	# Ticking `limited` before giving a range is not an error; it has no effect yet.
	joint.range_lower = null
	joint.range_upper = null
	check("limited with an empty range still compiles", world.compile())
	joint.range_lower = -0.5
	joint.range_upper = 0.5

	# Without the limit it hangs straight down.
	joint.limited = false
	world.compile()
	world.step(10000)  # 20 seconds
	check_near("with no limit it hangs straight down", body.rotation.z, -PI / 2, 0.02)
	world.free()


func test_slide_spring() -> void:
	# A 1 kg weight on a vertical slide with a spring. It settles where the
	# spring force equals its weight: stiffness * drop = mass * g.
	var parts := new_jointed_body("Spring")
	var world: Node3D = parts[0]
	var body: Node3D = parts[1]
	var joint: Node3D = parts[2]
	joint.type = SLIDE
	joint.stiffness = 100
	joint.damping = 20
	check("compiles", world.compile())
	world.step(2500)  # 5 seconds
	check_near("the weight settles on the spring", body.position.y, -G / 100.0, 0.001)
	check_near("without moving sideways", Vector2(body.position.x, body.position.z).length(), 0.0, 1e-6)
	world.free()
