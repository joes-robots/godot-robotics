extends "res://tests/test_base.gd"
## MujocoWorld's option_* properties: defaults, constants, validation, reverts,
## and saving to a scene.


func run() -> void:
	var world := make_world("World")
	await process_frame

	# Defaults are MuJoCo's, with vectors converted to Godot's Y-up frame.
	check_near("default timestep", world.option_timestep, 0.002, 1e-9)
	check_near("default impratio", world.option_impratio, 1.0, 1e-9)
	check_near("default tolerance", world.option_tolerance, 1e-8, 1e-12)
	check("default gravity is Godot -Y", world.option_gravity.is_equal_approx(Vector3(0, -9.81, 0)))
	check("default magnetic (MuJoCo 0,-0.5,0) in Godot frame", world.option_magnetic.is_equal_approx(Vector3(0, 0, 0.5)))
	check("default solver is Newton", world.option_solver == MujocoWorld.SOLVER_NEWTON)
	check_near("default contact override solimp dmin", world.get("option_o_solimp_dmin"), 0.9, 1e-9)
	check_near("default contact override spin friction", world.get("option_o_friction_spin"), 0.005, 1e-9)

	# Constants come from MuJoCo's enums and flag names.
	check("INTEGRATOR_RK4 == 1", MujocoWorld.INTEGRATOR_RK4 == 1)
	check("DISABLE_CONTACT == 16", MujocoWorld.DISABLE_CONTACT == 16)
	check("DISABLE_NATIVE_CCD == 1 << 17", MujocoWorld.DISABLE_NATIVE_CCD == 1 << 17)
	check("ENABLE_OVERRIDE == 1", MujocoWorld.ENABLE_OVERRIDE == 1)

	# The inspector lists the options in groups.
	var option_count := 0
	var groups: Array[String] = []
	for p in world.get_property_list():
		if p.usage & (PROPERTY_USAGE_GROUP | PROPERTY_USAGE_SUBGROUP) and p.hint_string.begins_with("option_"):
			groups.append(p.name)
		elif p.name.begins_with("option_"):
			option_count += 1
	check("38 option properties (got %d)" % option_count, option_count == 38)
	check("option groups %s" % [groups], groups == ["Options", "Flags", "Solver", "Collision", "Contact Override"])

	# Valid values are stored; reverts are MuJoCo's defaults.
	world.option_integrator = MujocoWorld.INTEGRATOR_RK4
	world.option_disableflags = MujocoWorld.DISABLE_CONTACT | MujocoWorld.DISABLE_GRAVITY
	world.option_o_solref_timeconst = 0.05
	world.option_gravity = Vector3(0, -1.62, 0)
	check("integrator set", world.option_integrator == MujocoWorld.INTEGRATOR_RK4)
	check("disableflags set", world.option_disableflags == 144)
	check("gravity set", world.option_gravity.is_equal_approx(Vector3(0, -1.62, 0)))
	check("options can revert", world.property_can_revert("option_timestep"))
	check("other properties are not ours to revert", not world.property_can_revert("name"))
	check("gravity reverts to default", world.property_get_revert("option_gravity").is_equal_approx(Vector3(0, -9.81, 0)))
	check("integrator reverts to Euler", world.property_get_revert("option_integrator") == MujocoWorld.INTEGRATOR_EULER)

	# Invalid values are rejected and leave the option unchanged (the errors they print are expected).
	print("(the next 3 errors are expected)")
	world.set("option_integrator", 7)
	world.set("option_disableflags", 1 << 25)
	world.set("option_timestep", "fast")
	check("bad integrator rejected", world.option_integrator == MujocoWorld.INTEGRATOR_RK4)
	check("unknown flag bits rejected", world.option_disableflags == 144)
	check("non-number timestep rejected", is_equal_approx(world.option_timestep, 0.002))

	# Only changed options are saved, and they load back.
	var packed := PackedScene.new()
	packed.pack(world)
	var path := "user://test_options_world.tscn"
	check("scene saved", ResourceSaver.save(packed, path) == OK)
	var text := FileAccess.get_file_as_string(path)
	check("unchanged options are not saved", not text.contains("option_timestep"))
	check("changed options are saved", text.contains("option_integrator") and text.contains("option_o_solref_timeconst"))
	var loaded: MujocoWorld = load(path).instantiate()
	root.add_child(loaded)
	await process_frame
	check("integrator reloaded", loaded.option_integrator == MujocoWorld.INTEGRATOR_RK4)
	check("gravity reloaded", loaded.option_gravity.is_equal_approx(Vector3(0, -1.62, 0)))
	check_near("solref timeconst reloaded", loaded.option_o_solref_timeconst, 0.05, 1e-9)
	DirAccess.remove_absolute(ProjectSettings.globalize_path(path))
