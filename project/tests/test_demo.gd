extends "res://tests/test_base.gd"
## The demo scene loads, compiles once and settles. The tumbling is chaotic, so
## only robust outcomes are checked, not exact poses.

var compiles := 0


func _on_compiled() -> void:
	compiles += 1


func run() -> void:
	var scene: Node = load("res://demos/basic.tscn").instantiate()
	var world: MujocoWorld = scene.get_node("MujocoWorld")
	world.model_compiled.connect(_on_compiled)
	root.add_child(scene)
	await run_until(world, 4.0)
	check("compiled once on load (got %d)" % compiles, compiles == 1)

	# Each falling object ends up resting on the floor.
	for item in [["Box", 0.5], ["Ball", 0.4], ["Capsule", 0.25]]:
		var body: Node3D = world.get_node(item[0])
		var y := body.global_position.y
		check("%s came down to the floor (y=%.3f)" % [item[0], y], y > item[1] - 0.02 and y < 1.5)

	# The box and ball stop. The capsule lands on its side and keeps rolling (MuJoCo's
	# default rolling friction is tiny), so for it only check that it stays on the floor.
	var positions := {}
	for item_name in ["Box", "Ball"]:
		positions[item_name] = world.get_node(item_name).global_position
	await run_until(world, 4.5)
	for item_name in ["Box", "Ball"]:
		var moved: float = world.get_node(item_name).global_position.distance_to(positions[item_name])
		check("%s at rest (moved %.4f m in 0.5 s)" % [item_name, moved], moved < 0.01)
	var capsule: Node3D = world.get_node("Capsule")
	check_near("capsule still lying on the floor", capsule.global_position.y, 0.25, 0.02)

	# The pendulum swings about a fixed pivot.
	var upper: Node3D = world.get_node("Pendulum")
	var lower: Node3D = world.get_node("Pendulum/Lower")
	check_near("pendulum pivot stays put", upper.global_position.distance_to(Vector3(2.5, 3.2, 0)), 0.0, 0.001)
	check_near("lower link stays 1 m from the upper bob", lower.global_position.distance_to(upper.global_transform * Vector3(0, -1, 0)), 0.0, 0.001)
