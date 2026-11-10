extends "res://tests/test_base.gd"
## Mesh geoms: any Godot Mesh simulates with its volume's mass and inertia (a
## BoxMesh like a box), shared meshes share one MuJoCo asset, and MJCF meshes
## (inline, STL, OBJ) import faithfully and can be saved as resource files.

const SAVE_DIR := "user://test_mesh_resources"


## A free body at p_position with one geom.
func free_body(world: MujocoWorld, body_name: String, p_position: Vector3) -> MujocoBody:
	var body: MujocoBody = add(world, MujocoBody.new(), body_name)
	body.position = p_position
	var joint: MujocoJoint = add(body, MujocoJoint.new(), "Free")
	joint.type = MujocoJoint.TYPE_FREE
	return body


func mesh_geom(body: Node, p_mesh: Mesh, geom_name := "Geom") -> MujocoGeom:
	var geom: MujocoGeom = add(body, MujocoGeom.new(), geom_name)
	geom.shape = MujocoGeom.SHAPE_MESH
	geom.mesh = p_mesh
	return geom


## The <mesh> assets of an MJCF string: name -> vertex attribute.
func mesh_assets(xml: String) -> Dictionary:
	var assets := {}
	var parser := XMLParser.new()
	parser.open_buffer(xml.to_utf8_buffer())
	while parser.read() == OK:
		if parser.get_node_type() == XMLParser.NODE_ELEMENT and parser.get_node_name() == "mesh":
			assets[parser.get_named_attribute_value_safe("name")] = parser.get_named_attribute_value_safe("vertex")
	return assets


func max_coordinate(vertex_attribute: String) -> float:
	var result := 0.0
	for number in vertex_attribute.split(" ", false):
		result = maxf(result, absf(number.to_float()))
	return result


func remove_dir(path: String) -> void:
	for file in DirAccess.get_files_at(path):
		DirAccess.remove_absolute(path.path_join(file))
	DirAccess.remove_absolute(path)


func run() -> void:
	# A BoxMesh geom has the mass and inertia of the same box: pushed and twisted
	# alike, both bodies move alike (an uneven box, so the axes must match too).
	var zw := make_world("ZeroGWorld")
	zw.option_gravity = Vector3.ZERO
	var size := Vector3(0.4, 1.0, 0.2)
	var boxed := free_body(zw, "Boxed", Vector3(-2, 0, 0))
	var box_geom: MujocoGeom = add(boxed, MujocoGeom.new(), "Geom")
	box_geom.size = size
	var meshed := free_body(zw, "Meshed", Vector3(2, 0, 0))
	var box_mesh := BoxMesh.new()
	box_mesh.size = size
	mesh_geom(meshed, box_mesh)
	var push := func() -> void:
		for body: MujocoBody in [boxed, meshed]:
			body.apply_central_force(Vector3(60, 20, 0))
			body.apply_torque(Vector3(1, 2, 3))
	physics_frame.connect(push)
	await run_until(zw, 0.5)
	physics_frame.disconnect(push)
	await physics_frame
	check("moving", boxed.get_linear_velocity().length() > 0.1 and boxed.get_angular_velocity().length() > 0.1)
	check("same linear velocity (%s vs %s)" % [meshed.get_linear_velocity(), boxed.get_linear_velocity()],
			meshed.get_linear_velocity().is_equal_approx(boxed.get_linear_velocity()))
	check("same angular velocity (%s vs %s)" % [meshed.get_angular_velocity(), boxed.get_angular_velocity()],
			(meshed.get_angular_velocity() - boxed.get_angular_velocity()).length() < 1e-4)
	check("same orientation", meshed.basis.is_equal_approx(boxed.basis))

	# The mesh is a MuJoCo asset; geoms sharing a Mesh share it. Editing the mesh
	# recompiles.
	var twin := mesh_geom(meshed, box_mesh, "Twin")
	twin.position = Vector3(0, 0, 1)
	await process_frame
	var assets := mesh_assets(zw.get_mjcf())
	check("one asset for the shared mesh (%s)" % [assets.keys()], assets.size() == 1)
	check("8 corners, merged from the faces", assets.values()[0].split(" ", false).size() == 24)
	box_mesh.size = Vector3(2, 2, 2)
	await process_frame
	check_near("editing the mesh recompiles", max_coordinate(mesh_assets(zw.get_mjcf()).values()[0]), 1.0, 1e-6)
	box_mesh.resource_name = "crate"
	twin.mesh = null
	await process_frame
	check("named after the resource", mesh_assets(zw.get_mjcf()).has("crate"))
	check("a Mesh geom without a mesh is left out", not mjcf_elements(zw.get_mjcf()).has("Meshed/Twin"))
	zw.queue_free()

	# On the floor, a mesh rests on its surface (contacts use its convex hull).
	var fw := make_world("FloorWorld", true)
	var dropped := free_body(fw, "Dropped", Vector3(0, 1, 0))
	var prism := PrismMesh.new()
	prism.size = Vector3(0.6, 0.4, 0.6)
	mesh_geom(dropped, prism)
	await run_until(fw, 1.5)
	check_near("the prism rests on its base", dropped.position.y, 0.2, 0.005)
	fw.queue_free()

	# Import: inline and file meshes, in the file's frame.
	var importer := MujocoImporter.new()
	var world := importer.import_file("res://tests/models/mesh.xml")
	check("mesh.xml imports", world != null)
	if world == null:
		return
	root.add_child(world)
	await process_frame
	check("nothing left out (report: %s)" % [importer.get_report()], importer.get_report().is_empty())
	var diffs := importer.verify(world, 1000)
	check("faithful: same compiled values and 1000 identical steps %s" % [diffs], diffs.is_empty())

	var tetra_a: MujocoGeom = world.get_node("tetra_body/tetra_a")
	var tetra_b: MujocoGeom = world.get_node("tetra_body/tetra_b")
	check("mesh shape from the class", tetra_a.shape == MujocoGeom.SHAPE_MESH and not tetra_a.overrides.has("shape"))
	check("one resource for a shared mesh", tetra_a.mesh != null and tetra_a.mesh == tetra_b.mesh)
	check("named after the MuJoCo mesh", tetra_a.mesh.resource_name == "tetra")
	check("the geom is where the file puts it", tetra_a.transform.is_equal_approx(Transform3D()))
	var aabb: AABB = tetra_a.mesh.get_aabb()
	# Scaled (0.2, 0.2, 0.3); MuJoCo's y is Godot's -z.
	check("vertices in the file's frame, scaled (%s)" % [aabb],
			aabb.position.is_equal_approx(Vector3(0, 0, -0.2)) and aabb.size.is_equal_approx(Vector3(0.2, 0.3, 0.2)))
	check("STL mesh", (world.get_node("wedge_body/wedge") as MujocoGeom).mesh.resource_name == "wedge")
	check("OBJ mesh", (world.get_node("oct_body/oct") as MujocoGeom).mesh.resource_name == "octahedron")
	check("a box fitted to a mesh stays a box", (world.get_node("oct_body/fitted") as MujocoGeom).shape == MujocoGeom.SHAPE_BOX)

	# Saved meshes become files the scene refers to.
	check("meshes saved", importer.save_meshes(SAVE_DIR) == OK)
	check("the geoms use the files", tetra_a.mesh.resource_path == SAVE_DIR.path_join("tetra.res"))
	var scene := PackedScene.new()
	check("packs into a scene", scene.pack(world) == OK)
	var copy: MujocoWorld = scene.instantiate()
	var copied: MujocoGeom = copy.get_node("tetra_body/tetra_a")
	check("the saved scene refers to the mesh file", copied.mesh.resource_path == SAVE_DIR.path_join("tetra.res"))
	copy.free()
	world.queue_free()
	await process_frame
	remove_dir(SAVE_DIR)
