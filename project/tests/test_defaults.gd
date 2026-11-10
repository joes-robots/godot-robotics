extends "res://tests/test_base.gd"
## Default classes (MujocoDefault): inheritance through main, class, child_class
## and nested classes; overrides and reverts on classes; the MJCF <default> tree;
## saving; and that the simulation uses the inherited values.


func add_class(parent: Node, class_name_: String) -> MujocoDefault:
	var cls: MujocoDefault = add(parent, MujocoDefault.new(), class_name_)
	return cls


## Attributes of the first <tag> inside <default class="p_class"> (or the root
## <default> for "main"), from an MJCF string.
func class_attributes(xml: String, p_class: String, tag: String) -> Dictionary:
	var parser := XMLParser.new()
	parser.open_buffer(xml.to_utf8_buffer())
	var stack: Array[String] = []
	while parser.read() == OK:
		if parser.get_node_type() == XMLParser.NODE_ELEMENT_END:
			if parser.get_node_name() == "default":
				stack.pop_back()
			continue
		if parser.get_node_type() != XMLParser.NODE_ELEMENT:
			continue
		var tag_name := parser.get_node_name()
		if tag_name == "default":
			var cls := parser.get_named_attribute_value_safe("class")
			stack.push_back(cls if not cls.is_empty() else "main")
			if parser.is_empty():
				stack.pop_back()
		elif tag_name == tag and not stack.is_empty() and stack.back() == p_class:
			var attributes := {}
			for i in parser.get_attribute_count():
				attributes[parser.get_attribute_name(i)] = parser.get_attribute_value(i)
			return attributes
	return {}


func run() -> void:
	var world := make_world("ClassWorld")
	world.option_gravity = Vector3.ZERO
	var main := add_class(world, "main")
	main.set("joint/damping", 0.5)
	var heavy := add_class(world, "Heavy")
	heavy.set("geom/density", 2000.0)
	var heavier := add_class(heavy, "Heavier")
	heavier.set("geom/density", 5000.0)
	heavier.set("geom/color", Color.RED)

	var body: MujocoBody = add(world, MujocoBody.new(), "Body")
	var free: MujocoJoint = add(body, MujocoJoint.new(), "Free")
	free.type = MujocoJoint.TYPE_FREE
	var plain: MujocoGeom = add(body, MujocoGeom.new(), "Plain")
	var classed: MujocoGeom = add(body, MujocoGeom.new(), "Classed")
	classed.default_class = &"Heavier"

	# Resolution: main, then the element's class and its parents.
	check("main has no parent class", main.get_parent_class() == null and main.is_main())
	check("a top-level class inherits from main", heavy.get_parent_class() == main)
	check("a nested class inherits from its parent", heavier.get_parent_class() == heavy)
	check("an element without a class uses main", plain.get_used_class() == main)
	check_near("main's damping reaches a joint", free.damping, 0.5, 0.0)
	check_near("no class sets density: base value", plain.density, 1000.0, 0.0)
	check_near("class value", classed.density, 5000.0, 0.0)
	check("class color", classed.color == Color.RED)
	check("inherited values aren't overrides", classed.overrides.is_empty())
	check_near("a class inherits from its parent", heavier.get("joint/damping"), 0.5, 0.0)

	# child_class applies to everything inside the body, unless something nearer says otherwise.
	body.child_class = &"Heavy"
	check_near("child_class", plain.density, 2000.0, 0.0)
	check_near("default_class beats child_class", classed.density, 5000.0, 0.0)
	var inner: MujocoBody = add(body, MujocoBody.new(), "Inner")
	inner.child_class = &"Heavier"
	var inner_geom: MujocoGeom = add(inner, MujocoGeom.new(), "Geom")
	check_near("the nearest child_class wins", inner_geom.density, 5000.0, 0.0)
	inner.queue_free()
	await process_frame

	# Overriding on an element, and reverting on a class.
	plain.density = 3000.0
	check_near("an element override beats its class", plain.density, 3000.0, 0.0)
	plain.set("density", null)
	heavier.set("geom/density", null)
	check_near("reverting a class value falls back to its parent class", classed.density, 2000.0, 0.0)
	check("the class's revert value is null when overridden", heavy.property_get_revert("geom/density") == null)
	check("class color still overridden", classed.color == Color.RED)

	# Size views on classes, and the drawn shape follows inherited values.
	heavier.set("geom/shape", MujocoGeom.SHAPE_SPHERE)
	heavier.set("geom/radius", 0.3)
	check_near("inherited radius", classed.radius, 0.3, 1e-12)
	check("class stores the half-size", heavier.overrides["geom"].get("size0") == 0.3)
	await process_frame
	var shape_instance: MeshInstance3D = classed.get_child(0, true)
	check("the drawn shape follows the class", shape_instance.mesh is SphereMesh and is_equal_approx((shape_instance.mesh as SphereMesh).radius, 0.3))

	# The MJCF has the class tree, and elements name their class.
	var xml := world.get_mjcf()
	check("main default has the joint damping", class_attributes(xml, "main", "joint").get("damping", "") == "0.5")
	check("Heavy has its density", class_attributes(xml, "Heavy", "geom").get("density", "") == "2000")
	check("Heavier is nested and has its shape", class_attributes(xml, "Heavier", "geom").get("type", "") == "sphere")
	var elements := mjcf_elements(xml)
	check("the geom names its class", elements.get("Body/Classed", {}).get("class", "") == "Heavier")
	check("and doesn't repeat inherited values", not elements.get("Body/Classed", {}).has("density"))
	check("the body has its childclass", elements.get("Body", {}).get("childclass", "") == "Heavy")

	# The simulation uses inherited values: two 1 m boxes in class Heavy weigh 2000 kg
	# each, so 4000 N for 1 s gives 1 m/s.
	classed.default_class = &"Heavy"
	heavier.queue_free()
	await process_frame
	check_near("total mass from the class", plain.density + classed.density, 4000.0, 0.0)
	var t0 := world.get_simulation_time()
	var push := func() -> void: body.apply_central_force(Vector3(4000, 0, 0))
	physics_frame.connect(push)
	await run_until(world, t0 + 1.0)
	physics_frame.disconnect(push)
	await physics_frame
	check_near("a = F / m with the class density", body.get_linear_velocity().x, world.get_simulation_time() - t0, 0.02)

	# Unknown classes fall back to main (and show a configuration warning in the editor).
	plain.default_class = &"Missing"
	check("unknown class falls back to main", plain.get_used_class() == main)
	plain.default_class = &""

	# Saving keeps class overrides.
	var scene := PackedScene.new()
	heavy.owner = null
	check("class packs", scene.pack(heavy) == OK)
	var copy: MujocoDefault = scene.instantiate()
	check("class overrides survive saving", copy.overrides == heavy.overrides)
	check_near("and resolve", copy.get("geom/density"), 2000.0, 0.0)
	var names := []
	for p in copy.get_property_list():
		names.append(p["name"])
	check("a class shows both actuator gains", names.has("actuator/kp") and names.has("actuator/kv"))
	copy.free()
	world.queue_free()
