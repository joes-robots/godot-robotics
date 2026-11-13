extends "res://tests/test_base.gd"
## The demo scene loads, compiles, and its shapes fall and come to rest.


func run() -> void:
	var scene: PackedScene = load("res://demos/shapes.tscn")
	check("demo scene loads", scene != null)
	var demo: Node3D = scene.instantiate()
	root.add_child(demo)
	var world: Node3D = demo.get_node("MujocoWorld")
	check("demo compiles", world.is_compiled() and world.get_compile_error().is_empty())

	world.paused = true
	world.step(20000)  # 40 seconds
	for shape_name in ["Sphere", "Box", "Capsule", "Cylinder"]:
		var body: Node3D = world.get_node(shape_name)
		check("%s has landed" % shape_name, body.position.y > 0.2 and body.position.y < 1.0)
	# The pendulum's limits (-120 to 30 degrees) leave room for it to hang straight down.
	var pendulum: Node3D = world.get_node("Pendulum")
	check_near("the pendulum hangs straight down", pendulum.rotation.z, -PI / 2, 0.05)
	check("no simulation error", not world.has_simulation_error())
