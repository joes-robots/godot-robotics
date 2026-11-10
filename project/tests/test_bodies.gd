extends "res://tests/test_base.gd"
## Body runtime API: velocities, teleport, forces and mocap bodies.
## A 1 m box at the default density of 1000 kg/m^3 has mass 1000 kg and, about any
## axis through its center, moment of inertia m (1 + 1) / 12 = 166.67 kg m^2.

const MASS := 1000.0
const INERTIA := MASS * 2.0 / 12.0


func zero_g_world(world_name: String) -> MujocoWorld:
	var world := make_world(world_name)
	world.option_gravity = Vector3.ZERO
	return world


func vectors_near(label: String, actual: Vector3, expected: Vector3, tolerance: float) -> void:
	var ok := actual.distance_to(expected) <= tolerance
	print("%s %s: got %s, expected %s (+/- %s)" % ["PASS" if ok else "FAIL", label, actual, expected, tolerance])
	if not ok:
		failures += 1


## Calls p_apply at the start of every physics frame (before the world steps) for
## p_duration simulated seconds. Returns how long the force actually acted.
func apply_each_frame(world: MujocoWorld, p_apply: Callable, p_duration: float) -> float:
	var t0 := world.get_simulation_time()
	physics_frame.connect(p_apply)
	await run_until(world, t0 + p_duration)
	physics_frame.disconnect(p_apply)
	# The last call happened at the start of this frame; let the world use it.
	await physics_frame
	return world.get_simulation_time() - t0


func run() -> void:
	# Velocities are global: with the world node rotated 90 degrees about Z, "down" in the
	# world's frame (local -Y) points along global +X.
	var tilted := make_world("TiltedWorld")
	tilted.position = Vector3(10, 0, 0)
	tilted.rotation_degrees = Vector3(0, 0, 90)
	var faller := make_free_box(tilted, 0.0)
	await run_until(tilted, 0.5)
	var t := tilted.get_simulation_time()
	vectors_near("falling velocity in global coordinates", faller.get_linear_velocity(), Vector3(G * t, 0, 0), 0.02)
	vectors_near("no spin", faller.get_angular_velocity(), Vector3.ZERO, 1e-6)
	tilted.queue_free()

	# set_linear_velocity / teleport.
	var w := zero_g_world("StateWorld")
	var box := make_free_box(w, 0.0)
	await process_frame
	box.teleport(Transform3D(Basis(Vector3.RIGHT, deg_to_rad(45)), Vector3(3, 4, 5)))
	check("teleport moves the node right away", box.global_transform.is_equal_approx(Transform3D(Basis(Vector3.RIGHT, deg_to_rad(45)), Vector3(3, 4, 5))))
	box.set_linear_velocity(Vector3(1, 0, 0))
	var start := box.global_position
	var t0 := w.get_simulation_time()
	await run_until(w, t0 + 1.0)
	vectors_near("1 m/s for 1 s moves 1 m", box.global_position - start, Vector3(1, 0, 0), 0.02)

	# set_angular_velocity is global too, even for a rotated body: the box is tilted 45 degrees
	# about X, so its local axes differ from the global ones.
	box.set_linear_velocity(Vector3.ZERO)
	box.set_angular_velocity(Vector3(0, 2, 0))
	vectors_near("angular velocity reads back (global)", box.get_angular_velocity(), Vector3(0, 2, 0), 1e-6)
	var basis0 := box.global_basis
	t0 = w.get_simulation_time()
	await run_until(w, t0 + 0.5)
	var turned := (box.global_basis * basis0.inverse()).get_rotation_quaternion()
	check_near("turned 1 rad about global Y in 0.5 s", turned.get_angle(), 1.0, 0.02)
	vectors_near("about global Y", turned.get_axis() * signf(turned.get_axis().y), Vector3.UP, 0.01)
	w.queue_free()

	# apply_central_force for one physics frame at a time: F = 1000 N on 1000 kg -> 1 m/s^2.
	var fw := zero_g_world("ForceWorld")
	var pushed := make_free_box(fw, 0.0)
	await process_frame
	var x0 := pushed.global_position.x
	var elapsed: float = await apply_each_frame(fw, func() -> void: pushed.apply_central_force(Vector3(MASS, 0, 0)), 1.0)
	check_near("a = F/m: velocity after 1 s", pushed.get_linear_velocity().x, elapsed, 0.02)
	check_near("a = F/m: distance after 1 s", pushed.global_position.x - x0, 0.5 * elapsed * elapsed, 0.02)
	# Without new calls the force stops: velocity stays constant.
	var v := pushed.get_linear_velocity().x
	await run_until(fw, fw.get_simulation_time() + 0.5)
	check_near("force only lasts one frame", pushed.get_linear_velocity().x, v, 1e-6)
	fw.queue_free()

	# apply_force off the center adds torque r x F: F along +Z at offset +X spins about -Y.
	var tw := zero_g_world("TorqueWorld")
	var spun := make_free_box(tw, 0.0)
	await process_frame
	elapsed = await apply_each_frame(tw, func() -> void: spun.apply_force(Vector3(0, 0, INERTIA), Vector3(1, 0, 0)), 1.0)
	vectors_near("off-center force spins (w = r x F t / I)", spun.get_angular_velocity(), Vector3(0, -elapsed, 0), 0.03)
	check_near("and pushes (v = F t / m)", spun.get_linear_velocity().z, INERTIA * elapsed / MASS, 0.005)
	tw.queue_free()

	# apply_torque, and forces from pre_step act for one step each (no build-up).
	var pw := zero_g_world("PreStepWorld")
	var twisted := make_free_box(pw, 0.0)
	var pushed2: MujocoBody = add(pw, MujocoBody.new(), "Pushed")
	pushed2.position = Vector3(5, 0, 0)
	var free_joint: MujocoJoint = add(pushed2, MujocoJoint.new(), "Free")
	free_joint.type = MujocoJoint.TYPE_FREE
	add(pushed2, MujocoGeom.new(), "Geom")
	var per_step := func() -> void:
		twisted.apply_torque(Vector3(0, INERTIA, 0))
		pushed2.apply_central_force(Vector3(MASS, 0, 0))
	await process_frame
	pw.pre_step.connect(per_step)
	t0 = pw.get_simulation_time()
	await run_until(pw, t0 + 1.0)
	pw.pre_step.disconnect(per_step)
	elapsed = pw.get_simulation_time() - t0
	vectors_near("torque from pre_step: w = T t / I", twisted.get_angular_velocity(), Vector3(0, elapsed, 0), 0.03)
	check_near("force from pre_step: v = F t / m", pushed2.get_linear_velocity().x, elapsed, 0.02)
	pw.queue_free()

	# A mocap body follows its node and pushes a free box along the floor.
	var mw := make_world("MocapWorld", true)
	var target := make_free_box(mw, 0.5)
	var pusher: MujocoBody = add(mw, MujocoBody.new(), "Pusher")
	pusher.mocap = true
	pusher.position = Vector3(-1.5, 0.5, 0)
	add(pusher, MujocoGeom.new(), "Geom")
	await process_frame
	var path_start := mw.get_simulation_time()
	var last_x := 0.0
	while mw.get_simulation_time() < path_start + 2.0:
		# 1 m/s to the right, set by the script each frame.
		last_x = -1.5 + (mw.get_simulation_time() - path_start)
		pusher.position.x = last_x
		await physics_frame
	check_near("the simulation doesn't move the mocap node", pusher.position.x, last_x, 1e-6)
	check("the pushed box moved right (x=%.3f)" % target.global_position.x, target.global_position.x > 0.3)
	check("the pushed box stays on the floor", absf(target.global_position.y - 0.5) < 0.05)

	# The free-joint setters need a free joint.
	print("(the next error is expected)")
	pusher.set_linear_velocity(Vector3.ONE)
	check("mocap body velocity cannot be set", pusher.get_linear_velocity() == Vector3.ZERO)
