extends "res://tests/test_base.gd"
## MujocoWorld: registration, the compile flow, and options as properties.


func run() -> void:
	check("MujocoWorld is registered", ClassDB.class_exists("MujocoWorld"))
	check("MujocoWorld is a Node3D", ClassDB.is_parent_class("MujocoWorld", "Node3D"))

	# Outside the tree nothing is compiled yet.
	var world: Node3D = ClassDB.instantiate("MujocoWorld")
	check("not compiled before entering the tree", not world.is_compiled())

	# Entering the tree compiles the (empty) model.
	add(root, world, "World")
	check("compiled after entering the tree", world.is_compiled())
	check("no compile error", world.get_compile_error().is_empty())

	# Changes that recompile leave a valid model behind.
	world.name = "Renamed"
	check("still compiled after a rename", world.is_compiled())
	check("compile() reports success", world.compile())
	check("still no compile error", world.get_compile_error().is_empty())

	await test_options(world)


func test_options(world: Node3D) -> void:
	var options := 0
	for property in world.get_property_list():
		if String(property.name).begins_with("option_"):
			options += 1
	check("has 24 option properties", options == 24)

	# Defaults are MuJoCo's own.
	check_near("default timestep", world.get("option_timestep"), 0.002, 1e-12)
	check("default solver is Newton", world.get("option_solver") == 2)
	# MuJoCo's default gravity is (0, 0, -9.81) in its Z-up frame: down. In Godot's
	# Y-up frame, down is -y.
	check("default gravity points down in Godot's frame",
			world.get("option_gravity").is_equal_approx(Vector3(0, -9.81, 0)))

	# Values read back as set, without recompiling being needed.
	world.set("option_timestep", 0.005)
	check_near("timestep can be changed", world.get("option_timestep"), 0.005, 1e-12)
	world.set("option_iterations", 50)
	check("iterations can be changed", world.get("option_iterations") == 50)
	world.set("option_integrator", 1)
	check("integrator can be changed", world.get("option_integrator") == 1)
	# A vector goes through both conversions (to MuJoCo on set, back on get).
	world.set("option_wind", Vector3(1, 2, 3))
	check("wind survives the round trip", world.get("option_wind").is_equal_approx(Vector3(1, 2, 3)))
	check("still compiled after option changes", world.is_compiled())
	check("recompiling keeps option values", world.compile() and is_equal_approx(world.get("option_timestep"), 0.005))

	# Invalid values are rejected (each prints an error) and leave the option alone.
	world.set("option_integrator", 99)
	check("out-of-range enum is rejected", world.get("option_integrator") == 1)
	world.set("option_timestep", "fast")
	check_near("wrong type is rejected", world.get("option_timestep"), 0.005, 1e-12)

	# Revert information, used by the inspector and when saving.
	check("changed option can revert", world.property_can_revert("option_timestep"))
	check_near("revert value is the default", world.property_get_revert("option_timestep"), 0.002, 1e-12)
	check("non-option cannot revert through us", world.property_get_revert("option_nonsense") == null)

	# Saving: a scene stores changed options, and a loaded copy has them.
	var scene := PackedScene.new()
	check("world packs into a scene", scene.pack(world) == OK)
	var copy: Node3D = scene.instantiate()
	check_near("timestep is saved", copy.get("option_timestep"), 0.005, 1e-12)
	check("wind is saved", copy.get("option_wind").is_equal_approx(Vector3(1, 2, 3)))
	check("unchanged option keeps its default", copy.get("option_solver") == 2)
	add(root, copy, "Copy")
	check("loaded copy compiles", copy.is_compiled())

	# Only changed options are written to the scene file.
	var path := "user://test_world_saved.tscn"
	ResourceSaver.save(scene, path)
	var text := FileAccess.get_file_as_string(path)
	check("changed option is in the scene file", text.contains("option_timestep"))
	check("unchanged option is not in the scene file", not text.contains("option_solver"))
	DirAccess.remove_absolute(ProjectSettings.globalize_path(path))
