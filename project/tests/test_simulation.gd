extends "res://tests/test_base.gd"
## Stepping and body sync, checked against closed-form physics.


func run() -> void:
	# Free fall, then resting on the floor.
	var fall_world := make_world("FallWorld", true)
	var box := make_free_box(fall_world, 2.0)
	await run_until(fall_world, 0.5)
	var t := fall_world.get_simulation_time()
	# Semi-implicit Euler trails the exact solution by about g*t*dt/2 (5 mm here).
	check_near("free fall: y = 2 - g t^2 / 2 at t=%.3f" % t, box.global_position.y, 2.0 - 0.5 * G * t * t, 0.01)
	await run_until(fall_world, 3.0)
	check_near("box rests on the floor at half its height", box.global_position.y, 0.5, 0.01)
	check_near("box rests level (deg)", rad_to_deg(box.global_basis.get_euler().length()), 0.0, 0.5)
	var rest_y := box.global_position.y
	await run_until(fall_world, 3.25)
	check_near("box stays at rest", absf(box.global_position.y - rest_y), 0.0, 0.001)

	# Changing an option at runtime applies to the running simulation without a reset.
	var zero_g_world := make_world("ZeroGWorld")
	var box2 := make_free_box(zero_g_world, 10.0)
	await run_until(zero_g_world, 0.25)
	zero_g_world.option_gravity = Vector3.ZERO
	var t1 := zero_g_world.get_simulation_time()
	var y1 := box2.global_position.y
	await run_until(zero_g_world, 0.75)
	var t2 := zero_g_world.get_simulation_time()
	var y2 := box2.global_position.y
	check("no reset: simulation time kept running (%.3f s)" % t2, t2 > 0.7)
	var expected_speed := G * t1
	check_near("constant speed after gravity off (m/s)", (y1 - y2) / (t2 - t1), expected_speed, expected_speed * 0.02)

	# running = false pauses.
	zero_g_world.running = false
	var paused_time := zero_g_world.get_simulation_time()
	var paused_y := box2.global_position.y
	for i in 10:
		await physics_frame
	check("paused: time frozen", zero_g_world.get_simulation_time() == paused_time)
	check("paused: body frozen", box2.global_position.y == paused_y)

	# Hinge pendulum: a small bob 1 m below the pivot, released at 10 degrees.
	var pendulum_world := make_world("PendulumWorld")
	var pendulum: MujocoBody = add(pendulum_world, MujocoBody.new(), "Pendulum")
	pendulum.position = Vector3(0, 2, 0)
	pendulum.rotation_degrees = Vector3(0, 0, 10)
	var hinge: MujocoJoint = add(pendulum, MujocoJoint.new(), "Hinge")
	hinge.rotation_degrees = Vector3(90, 0, 0) # axis (local Y) along Godot Z: swings in the X-Y plane
	var bob: MujocoGeom = add(pendulum, MujocoGeom.new(), "Bob")
	bob.shape = MujocoGeom.SHAPE_SPHERE
	bob.radius = 0.02
	bob.position = Vector3(0, -1, 0)
	await process_frame
	check_near("bob starts 10 deg out", bob.global_position.x, sin(deg_to_rad(10.0)), 0.001)

	var crossings: Array[float] = []
	var prev_x := bob.global_position.x
	var prev_t := pendulum_world.get_simulation_time()
	while pendulum_world.get_simulation_time() < 9.0:
		await physics_frame
		var x := bob.global_position.x
		var now := pendulum_world.get_simulation_time()
		if signf(x) != signf(prev_x) and prev_x != 0.0:
			crossings.append(prev_t + (now - prev_t) * prev_x / (prev_x - x))
		prev_x = x
		prev_t = now
	# Two zero crossings per period. Expected: 2 pi sqrt(L/g), corrected for the 10 degree
	# amplitude (1 + theta^2/16) and the sphere's own inertia.
	var period := 2.0 * (crossings[-1] - crossings[0]) / (crossings.size() - 1)
	var theta := deg_to_rad(10.0)
	var expected := TAU * sqrt(1.0 / G) * (1.0 + theta * theta / 16.0) * sqrt(1.0 + 0.4 * 0.02 * 0.02)
	check_near("pendulum period over %d crossings (s)" % crossings.size(), period, expected, expected * 0.01)

	# A runtime recompile resets the simulation to the original poses, not the simulated ones.
	var reset_world := make_world("ResetWorld", true)
	var box3 := make_free_box(reset_world, 2.0)
	await run_until(reset_world, 1.0)
	check("box has fallen before the reset", box3.global_position.y < 1.0)
	add(reset_world, MujocoGeom.new(), "AddedGeom").position = Vector3(5, 0.5, 0)
	await physics_frame
	check_near("structural change at runtime puts the box back at its start", box3.global_position.y, 2.0, 0.02)
