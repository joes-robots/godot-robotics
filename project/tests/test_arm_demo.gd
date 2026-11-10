extends "res://tests/test_base.gd"
## The robot arm demo: the servos track the moving target, and the mocap stirrer
## keeps the balls moving inside their pen.

var compiles := 0


func _on_compiled() -> void:
	compiles += 1


func run() -> void:
	var scene: Node3D = load("res://demos/robot_arm.tscn").instantiate()
	var world: MujocoWorld = scene.get_node("MujocoWorld")
	world.model_compiled.connect(_on_compiled)
	root.add_child(scene)
	var target: Node3D = scene.get_node("Target")
	var tip: Node3D = world.get_node("Base/Upper/Lower/Tip")
	var tip_sensor: MujocoSensor = tip.get_node("Position")
	var balls: Array[Node3D] = []
	var starts: Array[Vector3] = []
	for i in 4:
		balls.append(world.get_node("Ball%d" % (i + 1)))
		starts.append(balls[i].global_position)

	# Give the arm a moment to swing from its scene pose onto the circle.
	await run_until(world, 2.0)
	check("compiled once on load (got %d)" % compiles, compiles == 1)

	# Then the tip stays on the moving target, measured by the tip's position sensor.
	var worst := 0.0
	var sensor_mismatch := 0.0
	while world.get_simulation_time() < 8.0:
		await physics_frame
		var tip_position: Vector3 = tip_sensor.get_value()
		worst = maxf(worst, tip_position.distance_to(target.global_position))
		sensor_mismatch = maxf(sensor_mismatch, tip_position.distance_to(tip.global_position))
	check("tip tracks the target (worst error %.3f m)" % worst, worst < 0.05)
	check_near("tip sensor matches the tip site", sensor_mismatch, 0.0, 1e-4)

	var pivot: Node3D = world.get_node("Base/Upper")
	check_near("shoulder stays on its pedestal", pivot.global_position.distance_to(Vector3(1.5, 1.1, 0)), 0.0, 1e-4)

	# The stirrer follows the script and the balls get pushed around, staying in the pen.
	var stirrer: Node3D = world.get_node("Stirrer")
	check_near("stirrer on its circle", Vector2(stirrer.global_position.x + 3.0, stirrer.global_position.z).length(), 1.0, 1e-4)
	var moved := 0.0
	for i in balls.size():
		var p := balls[i].global_position
		moved += p.distance_to(starts[i])
		check("Ball%d inside the pen (%.2f, %.2f, %.2f)" % [i + 1, p.x, p.y, p.z], p.x > -5.0 and p.x < -1.0 and absf(p.z) < 2.0 and absf(p.y - 0.2) < 0.05)
	check("the balls were stirred (moved %.2f m in total)" % moved, moved > 1.0)
	scene.queue_free()
