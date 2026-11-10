extends "res://tests/test_base.gd"
## Class-able attributes: an element stores only its overrides and inherits the
## rest; setting null removes an override (the inspector's revert); scenes save the
## overrides. (Typed GDScript rejects `geom.density = null`, so tests use set().)


func run() -> void:
	var world := make_world("OverrideWorld")
	var body: MujocoBody = add(world, MujocoBody.new(), "Body")
	var joint: MujocoJoint = add(body, MujocoJoint.new(), "Hinge")
	var geom: MujocoGeom = add(body, MujocoGeom.new(), "Geom")

	# A new element overrides nothing and shows the inherited (base) values.
	check("new geom has no overrides", geom.overrides.is_empty())
	check_near("inherited density", geom.density, 1000.0, 0.0)

	# Setting a property creates an override; null removes it again.
	geom.density = 500.0
	check("setting a property overrides it", geom.overrides == {"density": 500.0})
	check_near("overridden density", geom.density, 500.0, 0.0)
	check("an override can revert", geom.property_can_revert("density"))
	check("its revert value is null", geom.property_get_revert("density") == null)
	geom.set("density", null)
	check("null removes the override", geom.overrides.is_empty())
	check_near("back to the inherited density", geom.density, 1000.0, 0.0)
	check("not overridden: the revert value is the current value", geom.property_get_revert("density") == 1000.0)

	# Ints are accepted for floats; a wrong type is rejected.
	joint.damping = 2
	check("int stored as float", typeof(joint.overrides["damping"]) == TYPE_FLOAT)
	print("(the next error is expected)")
	joint.overrides = {"damping": "lots"}
	check("wrong type rejected", joint.overrides.is_empty())

	# Sizes are stored as MuJoCo's half-sizes; radius, height and size are views.
	geom.shape = MujocoGeom.SHAPE_BOX
	geom.size = Vector3(2, 4, 6)
	check("box size stored as half x, z, y", geom.overrides.get("size0") == 1.0 and geom.overrides.get("size1") == 3.0 and geom.overrides.get("size2") == 2.0)
	geom.set("size", null)
	check("reverting size removes all three", not geom.overrides.has("size0") and not geom.overrides.has("size1") and not geom.overrides.has("size2"))

	# A capsule keeps a height set on it, whichever of height and radius comes first.
	geom.shape = MujocoGeom.SHAPE_CAPSULE
	geom.height = 2.0
	geom.radius = 0.3
	check_near("capsule height kept when the radius changes", geom.height, 2.0, 1e-12)
	check_near("stored as the half-length between the caps", geom.overrides["size1"], 0.7, 1e-12)
	# An inherited height follows MuJoCo: the caps grow, the middle stays.
	geom.set("height", null)
	var middle: float = geom.height - 2.0 * geom.radius
	geom.radius = 0.4
	check_near("inherited capsule height grows with the radius", geom.height - 2.0 * geom.radius, middle, 1e-12)

	# Plane size leaves MuJoCo's grid spacing (the third parameter) alone.
	var floor_geom: MujocoGeom = add(world, MujocoGeom.new(), "Floor")
	floor_geom.shape = MujocoGeom.SHAPE_PLANE
	floor_geom.size = Vector3(20, 0, 20)
	check("plane size sets two parameters", not floor_geom.overrides.has("size2"))
	check("plane size reads back with y = 0", floor_geom.size == Vector3(20, 0, 20))

	# Saved scenes keep overrides, even one equal to the base value, and nothing else.
	joint.damping = 0.0
	var scene := PackedScene.new()
	body.owner = null
	for node in [joint, geom]:
		node.owner = body
	check("scene packs", scene.pack(body) == OK)
	var copy: MujocoBody = scene.instantiate()
	var copy_joint: MujocoJoint = copy.get_node("Hinge")
	var copy_geom: MujocoGeom = copy.get_node("Geom")
	check("explicit damping = 0 survives saving", copy_joint.overrides == {"damping": 0.0})
	check("geom overrides survive saving", copy_geom.overrides == geom.overrides)
	check("and give the same values", copy_geom.radius == geom.radius and copy_geom.shape == MujocoGeom.SHAPE_CAPSULE)
	var saved_names: Array = []
	var state := scene.get_state()
	for i in state.get_node_property_count(1):
		saved_names.append(state.get_node_property_name(1, i))
	check("only overrides are saved for the joint (%s)" % [saved_names], saved_names.has("overrides") and not saved_names.has("damping") and not saved_names.has("range"))
	copy.free()

	# The model sees the same values as before the refactor.
	var elements := mjcf_elements(world.get_mjcf())
	check_numbers("capsule size in MJCF", elements.get("Body/Geom", {}).get("size", ""), "0.4 0.5")
	check("joint damping in MJCF (absent = 0)", not elements.get("Body/Hinge", {}).has("damping"))
	world.queue_free()
