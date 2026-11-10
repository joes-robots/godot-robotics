extends "res://tests/test_base.gd"
## The humanoid demo: the model converted from MJCF loads, falls like a rag doll,
## and starts over.


func run() -> void:
	var scene: Node3D = load("res://demos/humanoid.tscn").instantiate()
	root.add_child(scene)
	var world: MujocoWorld = scene.get_node("Humanoid")
	var torso: MujocoBody = world.get_node("torso")
	check("converted from its MJCF file", world.source_file == "res://demos/models/humanoid.xml")
	await physics_frame
	var start := torso.global_position
	check_near("starts standing", start.y, 1.282, 0.01)
	await run_until(world, 3.0)
	check("falls over (torso at %.2f m)" % torso.global_position.y, torso.global_position.y < 0.5)
	check("onto the floor, not through it", torso.global_position.y > 0.0)
	# The demo starts over after 4 simulated seconds.
	await run_until(world, 3.9)
	while world.get_simulation_time() > 1.0:
		await physics_frame
	check("starts over", torso.global_position.y > 1.0)
	scene.queue_free()
