extends SceneTree
## Base for the headless tests in this folder. Run them all with scripts/run_tests.sh.
##
## A test extends this script and overrides run(). Each check prints a PASS or
## FAIL line; at the end the test prints "RESULT: <n> failure(s)" and exits with
## the failure count, which the runner uses to decide pass or fail.

const G := 9.81

var failures := 0


func _initialize() -> void:
	# Let the root viewport become ready before building scenes in it.
	await process_frame
	await run()
	print("RESULT: %d failure(s)" % failures)
	quit(failures)


## Override in each test.
func run() -> void:
	pass


func check(label: String, ok: bool) -> void:
	print("%s %s" % ["PASS" if ok else "FAIL", label])
	if not ok:
		failures += 1


func check_near(label: String, actual: float, expected: float, tolerance: float) -> void:
	var ok := absf(actual - expected) <= tolerance
	print("%s %s: got %.5f, expected %.5f (+/- %.5f)" % ["PASS" if ok else "FAIL", label, actual, expected, tolerance])
	if not ok:
		failures += 1


## Compares space-separated numbers, as found in MJCF attributes.
func check_numbers(label: String, actual: String, expected: String, tolerance := 1e-5) -> void:
	var a := actual.split_floats(" ")
	var e := expected.split_floats(" ")
	var ok := a.size() == e.size()
	if ok:
		for i in a.size():
			ok = ok and absf(a[i] - e[i]) <= tolerance
	print("%s %s: got \"%s\", expected \"%s\"" % ["PASS" if ok else "FAIL", label, actual, expected])
	if not ok:
		failures += 1


func add(parent: Node, node: Node, node_name: String) -> Node:
	node.name = node_name
	parent.add_child(node)
	return node


func make_world(world_name: String, with_floor := false) -> MujocoWorld:
	var world: MujocoWorld = add(root, MujocoWorld.new(), world_name)
	if with_floor:
		var floor_geom: MujocoGeom = add(world, MujocoGeom.new(), "Floor")
		floor_geom.shape = MujocoGeom.SHAPE_PLANE
		floor_geom.size = Vector3(20, 0, 20)
	return world


## A body with a free joint and a 1 m box geom.
func make_free_box(world: MujocoWorld, height: float) -> MujocoBody:
	var box: MujocoBody = add(world, MujocoBody.new(), "Box")
	box.position = Vector3(0, height, 0)
	var joint: MujocoJoint = add(box, MujocoJoint.new(), "Free")
	joint.type = MujocoJoint.TYPE_FREE
	add(box, MujocoGeom.new(), "Geom")
	return box


## Waits for physics frames until the world has simulated `time` seconds.
## Needs --fixed-fps so every physics frame advances the same amount.
func run_until(world: MujocoWorld, time: float) -> void:
	while world.get_simulation_time() < time - 1e-9:
		await physics_frame


## Attributes of every named element in an MJCF string, keyed by name.
func mjcf_elements(xml: String) -> Dictionary:
	var elements := {}
	var parser := XMLParser.new()
	parser.open_buffer(xml.to_utf8_buffer())
	while parser.read() == OK:
		if parser.get_node_type() != XMLParser.NODE_ELEMENT:
			continue
		var element_name := parser.get_named_attribute_value_safe("name")
		if element_name.is_empty():
			continue
		var attributes := {}
		for i in parser.get_attribute_count():
			attributes[parser.get_attribute_name(i)] = parser.get_attribute_value(i)
		elements[element_name] = attributes
	return elements
