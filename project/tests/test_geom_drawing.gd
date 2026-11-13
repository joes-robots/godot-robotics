extends "res://tests/test_base.gd"
## MujocoGeom draws its own shape with an internal MeshInstance3D.
## This checks the mesh that gets built; what it looks like needs eyes.

var world: Node3D


func run() -> void:
	world = add(root, ClassDB.instantiate("MujocoWorld"), "World")
	var geom: Node3D = add(world, ClassDB.instantiate("MujocoGeom"), "Geom")

	check("the drawing is not an ordinary child", geom.get_child_count() == 0)
	check("it is one internal child", geom.get_child_count(true) == 1)
	var visual := geom.get_child(0, true) as MeshInstance3D
	check("which is a MeshInstance3D", visual != null)

	# Default geom: a sphere of radius 0.5, MuJoCo's default grey.
	var sphere := visual.mesh as SphereMesh
	check("default shape draws a sphere", sphere != null and is_equal_approx(sphere.radius, 0.5) and is_equal_approx(sphere.height, 1.0))
	var material := visual.material_override as StandardMaterial3D
	check("in the default colour", material != null and material.albedo_color.is_equal_approx(Color(0.5, 0.5, 0.5, 1)))

	# Godot meshes take full sizes; MuJoCo sizes are half-sizes.
	geom.shape = ClassDB.class_get_integer_constant("MujocoGeom", "SHAPE_BOX")
	geom.size = Vector3(1, 2, 3)
	var box := visual.mesh as BoxMesh
	check("box is drawn at twice the half-extents", box != null and box.size.is_equal_approx(Vector3(2, 4, 6)))

	geom.shape = ClassDB.class_get_integer_constant("MujocoGeom", "SHAPE_CAPSULE")
	geom.size = Vector3(0.1, 0.4, 0)
	var capsule := visual.mesh as CapsuleMesh
	# Half-length 0.4 is the cylindrical part: total height is 2 * 0.4 + 2 * 0.1.
	check("capsule height includes its caps", capsule != null and is_equal_approx(capsule.radius, 0.1) and is_equal_approx(capsule.height, 1.0))

	geom.shape = ClassDB.class_get_integer_constant("MujocoGeom", "SHAPE_CYLINDER")
	geom.size = Vector3(0.2, 0.3, 0)
	var cylinder := visual.mesh as CylinderMesh
	check("cylinder", cylinder != null and is_equal_approx(cylinder.top_radius, 0.2)
			and is_equal_approx(cylinder.bottom_radius, 0.2) and is_equal_approx(cylinder.height, 0.6))

	geom.shape = ClassDB.class_get_integer_constant("MujocoGeom", "SHAPE_PLANE")
	geom.size = Vector3(4, 0.1, 5)
	var plane := visual.mesh as PlaneMesh
	check("plane", plane != null and plane.size.is_equal_approx(Vector2(8, 10)))
	geom.size = Vector3(0, 0.1, 0)
	check("an infinite plane is drawn large", (visual.mesh as PlaneMesh).size.x >= 100)

	# Colour, including transparency.
	geom.rgba = Color(1, 0, 0, 0.5)
	material = visual.material_override as StandardMaterial3D
	check("colour follows rgba", material.albedo_color.is_equal_approx(Color(1, 0, 0, 0.5)))
	check("a translucent colour draws translucent", material.transparency == BaseMaterial3D.TRANSPARENCY_ALPHA)
	geom.rgba = null
	check("reverting rgba restores the default colour",
			(visual.material_override as StandardMaterial3D).albedo_color.is_equal_approx(Color(0.5, 0.5, 0.5, 1)))

	# The drawing doesn't disturb the model.
	check("the model still has exactly this geom", world.compile() and world.to_mjcf().count("<geom") == 1)

	# Turning the drawing off and on.
	check("draw_shape is on by default", geom.draw_shape)
	geom.draw_shape = false
	await process_frame
	check("turning draw_shape off removes the drawing", geom.get_child_count(true) == 0)
	geom.draw_shape = true
	check("turning it on again brings it back", geom.get_child_count(true) == 1)

	# Saving: the drawing isn't saved; a loaded geom makes its own.
	geom.shape = ClassDB.class_get_integer_constant("MujocoGeom", "SHAPE_BOX")
	geom.draw_shape = false
	var scene := PackedScene.new()
	scene.pack(geom)
	var copy: Node3D = scene.instantiate()
	check("draw_shape is saved", not copy.draw_shape)
	copy.draw_shape = true
	add(world, copy, "Copy")
	check("a loaded geom has one drawing", copy.get_child_count(true) == 1)
	check("of the saved shape", copy.get_child(0, true).mesh is BoxMesh)

	# Duplicating a geom must not double the drawing.
	var twin: Node3D = copy.duplicate()
	add(world, twin, "Twin")
	check("a duplicated geom has one drawing", twin.get_child_count(true) == 1)

	# Leaving and re-entering the tree keeps a single drawing.
	world.remove_child(copy)
	world.add_child(copy)
	check("re-entering the tree keeps one drawing", copy.get_child_count(true) == 1)
