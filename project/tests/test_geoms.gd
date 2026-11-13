extends "res://tests/test_base.gd"
## MujocoGeom: shapes and sizes, attributes stored as overrides, and what a
## failed compile leaves behind.

var world: Node3D
var body: Node3D


func run() -> void:
	check("MujocoGeom is registered", ClassDB.class_exists("MujocoGeom"))
	world = add(root, ClassDB.instantiate("MujocoWorld"), "World")
	body = add(world, ClassDB.instantiate("MujocoBody"), "Body")

	test_inspector_sections()
	test_shapes_and_sizes()
	test_overrides()
	test_placement()
	test_saving()
	await test_recompile_on_change()
	test_failed_compile()


func new_geom(parent: Node, geom_name: String) -> Node3D:
	return add(parent, ClassDB.instantiate("MujocoGeom"), geom_name)


## The compiled model's attributes for the geom or body called element_name.
func compiled(element_name: String) -> Dictionary:
	check("compiles", world.compile())
	return mjcf_elements(world.to_mjcf()).get(element_name, {})


## The inspector section (class name) that a node's property is listed under.
func inspector_section(node: Object, property_name: String) -> String:
	var section := ""
	for property in node.get_property_list():
		if property.usage & PROPERTY_USAGE_CATEGORY:
			section = property.name
		elif property.name == property_name:
			return section
	return ""


func test_inspector_sections() -> void:
	var geom := new_geom(body, "Sections")
	check("geom attributes are listed under MujocoGeom", inspector_section(geom, "size") == "MujocoGeom")
	check("as is draw_shape", inspector_section(geom, "draw_shape") == "MujocoGeom")
	var joint: Node3D = add(body, ClassDB.instantiate("MujocoJoint"), "SectionsJoint")
	check("joint attributes are listed under MujocoJoint", inspector_section(joint, "type") == "MujocoJoint")
	body.remove_child(geom)
	body.remove_child(joint)
	geom.free()
	joint.free()


func test_shapes_and_sizes() -> void:
	# A new geom works as it is: a sphere of radius 0.5.
	var ball := new_geom(body, "Ball")
	check("default shape is a sphere", ball.shape == ClassDB.class_get_integer_constant("MujocoGeom", "SHAPE_SPHERE"))
	check("default size", ball.size == Vector3(0.5, 0.5, 0.5))
	var attributes := compiled("Body/Ball")
	check("geom is named by its path", not attributes.is_empty())
	check_numbers("sphere radius", attributes.get("size", ""), "0.5")

	# Box: half-extents along Godot's axes. Godot's y (up) is MuJoCo's z.
	var box := new_geom(body, "Box")
	box.shape = ClassDB.class_get_integer_constant("MujocoGeom", "SHAPE_BOX")
	box.size = Vector3(1, 3, 2)
	attributes = compiled("Body/Box")
	check("box type", attributes.get("type", "") == "box")
	check_numbers("box size is reordered for MuJoCo", attributes.get("size", ""), "1 2 3")

	# Capsule: radius and half-length, no reordering.
	var capsule := new_geom(body, "Capsule")
	capsule.shape = ClassDB.class_get_integer_constant("MujocoGeom", "SHAPE_CAPSULE")
	capsule.size = Vector3(0.1, 0.4, 0)
	attributes = compiled("Body/Capsule")
	check("capsule type", attributes.get("type", "") == "capsule")
	check_numbers("capsule radius and half-length", attributes.get("size", ""), "0.1 0.4")

	var cylinder := new_geom(body, "Cylinder")
	cylinder.shape = ClassDB.class_get_integer_constant("MujocoGeom", "SHAPE_CYLINDER")
	cylinder.size = Vector3(0.2, 0.3, 0)
	attributes = compiled("Body/Cylinder")
	check("cylinder type", attributes.get("type", "") == "cylinder")
	check_numbers("cylinder radius and half-length", attributes.get("size", ""), "0.2 0.3")

	# Plane: x and z are the half-extents, y is the grid spacing.
	var plane := new_geom(world, "Floor")
	plane.shape = ClassDB.class_get_integer_constant("MujocoGeom", "SHAPE_PLANE")
	plane.size = Vector3(20, 0.1, 10)
	attributes = compiled("Floor")
	check("plane type", attributes.get("type", "") == "plane")
	check_numbers("plane size is reordered for MuJoCo", attributes.get("size", ""), "20 10 0.1")


func test_overrides() -> void:
	var geom := new_geom(body, "Overrides")

	# Nothing set: values are MuJoCo's defaults and nothing is stored.
	check("no overrides on a new geom", geom.get_overrides().is_empty())
	check_near("inherited density is MuJoCo's default", geom.density, 1000.0, 1e-9)
	check("inherited friction is MuJoCo's default", geom.friction.is_equal_approx(Vector3(1, 0.005, 0.0001)))
	check("inherited rgba is MuJoCo's default", geom.rgba.is_equal_approx(Color(0.5, 0.5, 0.5, 1)))
	check("inherited contype is MuJoCo's default", geom.contype == 1 and geom.conaffinity == 1)
	check_near("inherited mass means 'use density'", geom.mass, -1.0, 1e-9)
	var attributes := compiled("Body/Overrides")
	check("unset attributes are not written to the model", not attributes.has("density") and not attributes.has("mass"))

	# Setting stores an override, and the model gets it.
	geom.density = 500
	check("setting stores an override", geom.has_override("density") and geom.get_overrides() == {"density": 500.0})
	check("an int given for a float attribute is stored as a float", typeof(geom.get_overrides()["density"]) == TYPE_FLOAT)
	check("an overridden attribute can be reverted", geom.property_can_revert("density"))
	check("other attributes can't", not geom.property_can_revert("friction"))
	check_numbers("override reaches the model", compiled("Body/Overrides").get("density", ""), "500")

	# Removing the override, by assigning null (what the inspector's revert does).
	geom.density = null
	check("assigning null removes the override", not geom.has_override("density"))
	check_near("the inherited value is back", geom.density, 1000.0, 1e-9)

	# An explicit value equal to the default is still an override.
	geom.density = 1000
	check("a value equal to the default is still stored", geom.has_override("density"))
	geom.clear_override("density")
	check("clear_override removes it", not geom.has_override("density"))

	# The other attributes.
	geom.mass = 2.5
	geom.rgba = Color(1, 0, 0, 0.5)
	geom.friction = Vector3(0.7, 0.01, 0.001)
	geom.contype = 2
	geom.conaffinity = 5
	attributes = compiled("Body/Overrides")
	check_numbers("mass", attributes.get("mass", ""), "2.5")
	check_numbers("rgba", attributes.get("rgba", ""), "1 0 0 0.5")
	check_numbers("friction", attributes.get("friction", ""), "0.7 0.01 0.001")
	check_numbers("contype", attributes.get("contype", ""), "2")
	check_numbers("conaffinity", attributes.get("conaffinity", ""), "5")

	# A wrong type is rejected (prints an error) and nothing changes.
	geom.set("density", "heavy")
	check("wrong type is rejected", not geom.has_override("density"))


func test_placement() -> void:
	# A geom's transform is its pose in its body. Godot (0, 1, -2) is MuJoCo (0, 2, 1).
	var geom := new_geom(body, "Placed")
	geom.position = Vector3(0, 1, -2)
	check_numbers("geom position is converted", compiled("Body/Placed").get("pos", ""), "0 2 1")

	# Elements can't be nested inside a geom.
	add(geom, ClassDB.instantiate("MujocoBody"), "Lost")
	check("a body under a geom is not in the model", compiled("Body/Placed/Lost").is_empty() and not world.to_mjcf().contains("Lost"))


func test_saving() -> void:
	var geom: Node3D = ClassDB.instantiate("MujocoGeom")
	geom.shape = ClassDB.class_get_integer_constant("MujocoGeom", "SHAPE_BOX")
	geom.size = Vector3(1, 2, 3)
	geom.density = 1000  # equal to the default, but explicitly set

	var scene := PackedScene.new()
	check("geom packs into a scene", scene.pack(geom) == OK)
	var copy: Node3D = scene.instantiate()
	check("overrides survive saving", copy.get_overrides() == geom.get_overrides())
	check("an explicit default-valued override survives", copy.has_override("density"))
	check("unset attributes stay unset", not copy.has_override("friction"))

	var path := "user://test_geoms_saved.tscn"
	ResourceSaver.save(scene, path)
	var text := FileAccess.get_file_as_string(path)
	check("the scene file stores the overrides dictionary", text.contains("overrides = "))
	check("and not the individual attributes", not text.contains("density = ") and not text.contains("size = "))
	DirAccess.remove_absolute(ProjectSettings.globalize_path(path))
	geom.free()
	copy.free()


func test_recompile_on_change() -> void:
	var geom := new_geom(body, "Changing")
	world.compile()
	geom.density = 250
	await process_frame
	check_numbers("changing an attribute recompiles", mjcf_elements(world.to_mjcf())["Body/Changing"].get("density", ""), "250")


func test_failed_compile() -> void:
	check("starts from a good model", world.compile() and world.get_compile_error().is_empty())
	var good_model: String = world.to_mjcf()

	# A zero-size sphere is not a valid geom.
	var bad := new_geom(body, "Bad")
	bad.size = Vector3.ZERO
	check("compile reports failure", not world.compile())
	check("the error is available", world.get_compile_error().contains("size"))
	check("the last good model is still there", world.is_compiled() and world.to_mjcf() == good_model)

	# Fixing the problem compiles again and clears the error.
	bad.size = Vector3(0.1, 0, 0)
	check("compiles after the fix", world.compile())
	check("the error is cleared", world.get_compile_error().is_empty())
	check("the new model has the geom", world.to_mjcf().contains("Body/Bad"))
