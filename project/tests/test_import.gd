extends "res://tests/test_base.gd"
## MujocoImporter: an MJCF file becomes the nodes you would build by hand, and
## whatever can't be represented is reported.

var HINGE: int = ClassDB.class_get_integer_constant("MujocoJoint", "TYPE_HINGE")
var SLIDE: int = ClassDB.class_get_integer_constant("MujocoJoint", "TYPE_SLIDE")
var FREE: int = ClassDB.class_get_integer_constant("MujocoJoint", "TYPE_FREE")
var PLANE: int = ClassDB.class_get_integer_constant("MujocoGeom", "SHAPE_PLANE")
var BOX: int = ClassDB.class_get_integer_constant("MujocoGeom", "SHAPE_BOX")
var CAPSULE: int = ClassDB.class_get_integer_constant("MujocoGeom", "SHAPE_CAPSULE")
var CYLINDER: int = ClassDB.class_get_integer_constant("MujocoGeom", "SHAPE_CYLINDER")


func run() -> void:
	check("MujocoImporter is registered", ClassDB.class_exists("MujocoImporter"))
	test_arm()
	test_unsupported()
	test_failures()
	test_import_string()
	test_verify()


func report_kinds(importer: Object) -> Dictionary:
	var counts := {}
	for entry in importer.get_report():
		counts[entry.kind] = counts.get(entry.kind, 0) + 1
	return counts


func report_mentions(importer: Object, text: String) -> bool:
	return importer.get_report_text().contains(text)


func test_arm() -> void:
	var importer: Object = ClassDB.instantiate("MujocoImporter")
	var world: Node3D = importer.import_file("res://tests/models/arm.xml")
	check("imports", world != null and importer.get_error().is_empty())
	if world == null:
		print(importer.get_error())
		return
	check("the world is named after the model", world.name == "arm")
	check("it is not in the tree yet", not world.is_inside_tree())

	# Options: only the ones the file changes.
	check_near("timestep option", world.get("option_timestep"), 0.004, 1e-12)
	# MuJoCo's "0 0 -5" is 5 m/s^2 downwards: Godot's -y.
	check("gravity option, converted", world.get("option_gravity").is_equal_approx(Vector3(0, -5, 0)))

	# Structure: the same tree as the file.
	check("world-level nodes", world.get_child_count() == 5)
	var upper: Node3D = world.get_node_or_null("upper")
	var lower: Node3D = world.get_node_or_null("upper/lower")
	check("bodies nest as in the file", upper != null and lower != null
			and upper.get_class() == "MujocoBody" and lower.get_class() == "MujocoBody")
	check("geoms outside any body sit under the world", world.get_node("floor").get_class() == "MujocoGeom")
	check("an unnamed geom gets a default name", world.get_node_or_null("Geom") != null)
	check("joints sit under their body", world.get_node("upper/shoulder").get_class() == "MujocoJoint")
	check("every node is owned by the world, so it saves with it", lower.owner == world and world.get_node("upper/shoulder").owner == world)

	# Poses. MuJoCo (0, 0, 1) is Godot (0, 1, 0).
	check("body position is converted", upper.position.is_equal_approx(Vector3(0, 1, 0)))
	# euler="0 90 0" is a quarter turn about MuJoCo's y, which is Godot's -z.
	check("an euler orientation is resolved", upper.quaternion.is_equal_approx(Quaternion(Vector3(0, 0, -1), PI / 2)))
	check("child body position is relative to its parent", lower.position.is_equal_approx(Vector3(0, 0.4, 0)))

	# Geoms.
	var floor_geom: Node3D = world.get_node("floor")
	check("plane", floor_geom.shape == PLANE and floor_geom.plane_half_size == Vector2(2, 3) and is_equal_approx(floor_geom.grid_spacing, 0.1))
	var box: Node3D = world.get_node("Geom")
	# size="0.1 0.2 0.3" along MuJoCo x, y, z is (0.1, 0.3, 0.2) along Godot's.
	check("box half-extents are laid out along Godot's axes", box.shape == BOX and box.half_extents.is_equal_approx(Vector3(0.1, 0.3, 0.2)))
	check("box position is converted", box.position.is_equal_approx(Vector3(1, 0.3, 0)))

	var upper_geom: Node3D = world.get_node("upper/upper_geom")
	# fromto="0 0 0  0 0 0.4": a capsule 0.4 long, so half-length 0.2, centred
	# 0.2 along the body's z, which is Godot's y.
	check("a capsule given by its ends is resolved", upper_geom.shape == CAPSULE
			and is_equal_approx(upper_geom.radius, 0.05) and is_equal_approx(upper_geom.half_length, 0.2)
			and upper_geom.position.is_equal_approx(Vector3(0, 0.2, 0)))
	check("a value from a default class is imported", upper_geom.color.is_equal_approx(Color(0.2, 0.4, 0.8, 1)))
	check("friction", is_equal_approx(upper_geom.friction_sliding, 0.8) and is_equal_approx(upper_geom.friction_torsional, 0.01)
			and is_equal_approx(upper_geom.friction_rolling, 0.002))
	var lower_geom: Node3D = world.get_node("upper/lower/Geom")
	check("mass and collision masks", is_equal_approx(lower_geom.mass, 0.5) and lower_geom.contype == 2 and lower_geom.conaffinity == 3)
	check_near("density", world.get_node("upper/lower/tip/Geom").density, 500.0, 1e-9)

	# Only what differs from the defaults is stored.
	var ball_geom: Node3D = world.get_node("ball/Geom")
	check("a plain geom stores only what the file changed", ball_geom.get_overrides() == {"radius": 0.1})
	check("unset attributes are not stored", not upper_geom.has_override("density") and not upper_geom.has_override("contype"))

	# Joints.
	var shoulder: Node3D = world.get_node("upper/shoulder")
	check("hinge", shoulder.type == HINGE and not shoulder.has_override("type"))
	# axis="0 1 0" in MuJoCo is Godot (0, 0, -1).
	check("joint axis is converted", shoulder.axis.is_equal_approx(Vector3(0, 0, -1)))
	check("the joint node is left unrotated", shoulder.basis.is_equal_approx(Basis.IDENTITY))
	# range="-45 90" is in degrees in the file.
	check("limits are in radians", shoulder.limited and is_equal_approx(shoulder.range_lower, -PI / 4) and is_equal_approx(shoulder.range_upper, PI / 2))
	check("damping from the default class, and armature", is_equal_approx(shoulder.damping, 0.5) and is_equal_approx(shoulder.armature, 0.01))
	var elbow: Node3D = world.get_node("upper/lower/elbow")
	# range="0 120": a lower limit of 0 is MuJoCo's default, so only the upper is stored.
	check("a second hinge", elbow.type == HINGE and elbow.limited and is_equal_approx(elbow.range_upper, deg_to_rad(120)) and not elbow.has_override("range_lower"))
	var extend: Node3D = world.get_node("upper/lower/tip/extend")
	# axis="0 0 1" is MuJoCo's default axis, so nothing is stored for it.
	check("slide", extend.type == SLIDE and extend.axis.is_equal_approx(Vector3(0, 1, 0)) and not extend.has_override("axis"))
	check("a slide's limits are lengths, unconverted", is_equal_approx(extend.range_lower, -0.1) and is_equal_approx(extend.range_upper, 0.2))
	check("stiffness and springref", is_equal_approx(extend.stiffness, 30) and is_equal_approx(extend.springref, 0.05))
	check("free joint", world.get_node("ball/ball_free").type == FREE)
	check("an unlimited joint is not marked limited", not world.get_node("ball/ball_free").has_override("limited"))

	# Mocap.
	check("mocap body", world.get_node("target").mocap == true and not upper.has_override("mocap"))
	check("cylinder", world.get_node("target/Geom").shape == CYLINDER)

	# Names: the compiled model keeps the file's names.
	check("a top-level element needs no name override", upper.get("mujoco_name") == "" and upper.get_mujoco_name() == "upper")
	check("a nested element keeps its name by override", lower.get("mujoco_name") == "lower" and shoulder.get("mujoco_name") == "shoulder")
	add(root, world, "arm")
	check("the imported world compiles", world.is_compiled() and world.get_compile_error().is_empty())
	var elements := mjcf_elements(world.to_mjcf())
	for original_name in ["floor", "upper", "lower", "tip", "shoulder", "elbow", "extend", "upper_geom", "ball", "ball_free", "target"]:
		check("the compiled model has '%s' under its original name" % original_name, elements.has(original_name))

	# Report: the default class, and nothing else.
	var kinds := report_kinds(importer)
	check("the report notes the flattened default class", kinds.get("flattened_default", 0) == 1 and report_mentions(importer, "1 default class was flattened"))
	check("and nothing was skipped", kinds.get("skipped_element", 0) == 0 and kinds.get("skipped_attribute", 0) == 0)

	# It behaves: the free ball falls.
	world.paused = true
	var ball: Node3D = world.get_node("ball")
	var start_height: float = ball.position.y
	world.step(100)  # 0.4 s at this model's timestep
	check_near("the imported model simulates", ball.position.y, start_height - 0.5 * 5.0 * 0.4 * 0.4, 0.02)
	world.free()


func test_unsupported() -> void:
	var importer: Object = ClassDB.instantiate("MujocoImporter")
	var world: Node3D = importer.import_file("res://tests/models/unsupported.xml")
	check("a file with unsupported features still imports", world != null)
	if world == null:
		print(importer.get_error())
		return

	# What is supported comes through.
	check("the body, joint and box are imported", world.get_node_or_null("base") != null
			and world.get_node_or_null("base/pivot") != null and world.get_node_or_null("base/pad") != null)
	# What isn't is left out, not approximated.
	check("the ellipsoid geom is skipped, not replaced", world.get_node_or_null("base/egg") == null and world.get_node("base").get_child_count() == 2)

	var text: String = importer.get_report_text()
	check("reports the skipped geom by name", text.contains("egg: ") and text.contains("ellipsoid"))
	check("reports the explicit inertial", text.contains("base: ") and text.contains("<inertial>"))
	check("reports gravity compensation", text.contains("gravcomp"))
	check("reports unsupported geom attributes", text.contains("pad: ") and text.contains("condim") and text.contains("margin"))
	check("reports unsupported joint attributes", text.contains("pivot: ") and text.contains("actuatorfrcrange"))
	check("reports option flags", text.contains("<flag>"))
	check("counts skipped actuators", text.contains("2 actuators were skipped"))
	check("counts the skipped sensor and site", text.contains("1 sensor was skipped") and text.contains("1 site was skipped"))

	# The report is data too.
	var report: Array = importer.get_report()
	var well_formed := true
	for entry in report:
		well_formed = well_formed and entry.has("kind") and entry.has("element") and entry.has("message")
	check("every report entry has a kind, an element and a message", well_formed and not report.is_empty())
	var kinds := report_kinds(importer)
	check("entries are classified", kinds.get("skipped_element", 0) == 4 and kinds.get("skipped_attribute", 0) == 5)
	world.free()


func test_failures() -> void:
	var importer: Object = ClassDB.instantiate("MujocoImporter")
	check("a missing file gives null", importer.import_file("res://tests/models/no_such_file.xml") == null)
	check("and an error", importer.get_error().contains("no_such_file.xml"))
	check("malformed XML gives null", importer.import_string("<mujoco><worldbody>") == null and not importer.get_error().is_empty())
	# Parses, but a zero-size sphere doesn't compile.
	check("a model that doesn't compile gives null", importer.import_string("<mujoco><worldbody><geom size='0'/></worldbody></mujoco>") == null)
	check("with MuJoCo's reason", importer.get_error().contains("size"))
	# A good import after a failed one clears the error.
	var world: Node3D = importer.import_string("<mujoco><worldbody><geom size='1'/></worldbody></mujoco>")
	check("the error is cleared by a later success", world != null and importer.get_error().is_empty())
	world.free()


func test_import_string() -> void:
	var importer: Object = ClassDB.instantiate("MujocoImporter")
	var world: Node3D = importer.import_string("""
		<mujoco>
			<worldbody>
				<body name="a"><geom size="0.2"/>
					<body name="a2"><geom size="0.1"/></body>
				</body>
			</worldbody>
		</mujoco>""")
	check("imports from text", world != null and world.get_node_or_null("a/a2/Geom") != null)
	check("an unnamed model gets a default world name", world.name == "MujocoWorld")
	check("an empty report for a fully supported model", importer.get_report().is_empty() and importer.get_report_text().is_empty())

	# Saving the result as a scene and loading it gives the same tree.
	var scene := PackedScene.new()
	check("the imported world packs into a scene", scene.pack(world) == OK)
	var copy: Node3D = scene.instantiate()
	check("the saved scene has the whole tree", copy.get_node_or_null("a/a2/Geom") != null
			and copy.get_node("a/a2").get("mujoco_name") == "a2"
			and is_equal_approx(copy.get_node("a/a2/Geom").radius, 0.1))
	world.free()
	copy.free()


func difference_fields(differences: Array) -> Array:
	var fields := []
	for entry in differences:
		fields.append(entry.field)
	return fields


func test_verify() -> void:
	var importer: Object = ClassDB.instantiate("MujocoImporter")
	var path := "res://tests/models/arm.xml"

	# A fully supported model matches its original in every field, and behaves the same.
	var world: Node3D = importer.import_file(path)
	add(root, world, "VerifyArm")
	world.paused = true
	var differences: Array = importer.verify_file(world, path)
	check("the arm model matches its original exactly", differences.is_empty())
	check("which reads as 'No differences.'", importer.differences_to_text(differences) == "No differences.")
	check("verifying leaves the world's own simulation alone", is_equal_approx(world.get_time(), 0.0))

	# verify() notices changes. Each of these alters the imported tree, then checks again.
	world.get_node("ball/Geom").radius = 0.2
	world.compile()
	var fields := difference_fields(importer.verify_file(world, path))
	check("a changed geom size is found", fields.has("geom_size"))
	check("and so is the mass that follows from it", fields.has("body_mass"))
	world.get_node("ball/Geom").radius = 0.1

	world.get_node("upper/shoulder").damping = 0.7
	world.compile()
	fields = difference_fields(importer.verify_file(world, path))
	# The arm swings under gravity, so its damping changes the motion too.
	check("a changed joint attribute is found, with its effect and nothing else", fields == ["dof_damping", "simulation"])
	world.get_node("upper/shoulder").damping = 0.5

	world.get_node("upper/lower/tip/extend").stiffness = 300
	world.compile()
	fields = difference_fields(importer.verify_file(world, path))
	check("a change that alters the motion shows up in the simulation", fields.has("jnt_stiffness") and fields.has("simulation"))
	world.get_node("upper/lower/tip/extend").stiffness = 30

	world.get_node("upper/lower").set("mujoco_name", "forearm")
	world.compile()
	differences = importer.verify_file(world, path)
	check("a changed name is found", difference_fields(differences) == ["body names"]
			and differences[0].message.contains("'lower' is 'forearm'"))
	world.get_node("upper/lower").set("mujoco_name", "lower")

	world.set("option_timestep", 0.001)
	fields = difference_fields(importer.verify_file(world, path))
	check("a changed option is found", fields.has("opt"))
	world.set("option_timestep", 0.004)

	world.get_node("ball").position += Vector3(0, 0.01, 0)
	world.compile()
	fields = difference_fields(importer.verify_file(world, path))
	check("a moved body is found", fields.has("body_pos"))
	world.get_node("ball").position -= Vector3(0, 0.01, 0)

	world.compile()
	check("with everything put back, it matches again", importer.verify_file(world, path).is_empty())

	# Removing something changes the counts, reported once each.
	var ball: Node3D = world.get_node("ball")
	world.remove_child(ball)
	world.compile()
	fields = difference_fields(importer.verify_file(world, path))
	check("a missing body is reported as a difference in counts", fields.has("nbody") and fields.has("njnt") and fields.has("ngeom"))
	check("without a cascade of array differences", not fields.has("body_pos") and not fields.has("geom_size"))
	ball.free()
	world.free()

	# A model with unsupported features imports, and verify says what is different.
	path = "res://tests/models/unsupported.xml"
	world = importer.import_file(path)
	add(root, world, "VerifyUnsupported")
	differences = importer.verify_file(world, path)
	fields = difference_fields(differences)
	check("the skipped geom shows as a difference in counts", fields.has("ngeom"))
	check("the skipped actuators too", fields.has("nu"))
	check("and the explicit inertial as a difference in mass", fields.has("body_mass"))
	var well_formed := true
	for entry in differences:
		well_formed = well_formed and entry.has("field") and entry.has("message") and entry.has("count") and entry.has("max_difference")
	check("every difference has a field, a message, a count and a size", well_formed)

	# The check itself can fail.
	check("verifying against a missing file reports an error",
			difference_fields(importer.verify_file(world, "res://tests/models/no_such_file.xml")) == ["error"])
	var loose: Node3D = importer.import_string("<mujoco><worldbody><geom size='1'/></worldbody></mujoco>")
	differences = importer.verify_string(loose, "<mujoco><worldbody><geom size='1'/></worldbody></mujoco>")
	check("verifying a world that isn't compiled reports an error", difference_fields(differences) == ["error"] and differences[0].message.contains("scene tree"))
	add(root, loose, "Loose")
	check("verify_string works from text", importer.verify_string(loose, "<mujoco><worldbody><geom size='1'/></worldbody></mujoco>").is_empty())
	loose.free()
	world.free()
