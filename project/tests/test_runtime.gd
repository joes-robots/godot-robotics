extends "res://tests/test_base.gd"
## Runtime control: reset(), step(), the pre_step signal, and joint state.

var pre_steps := 0


func _count_pre_step() -> void:
	pre_steps += 1


## A pendulum: hinge about Godot Z at the body origin, a small bob 1 m below.
func make_pendulum(world: MujocoWorld, swing_deg: float) -> MujocoBody:
	var body: MujocoBody = add(world, MujocoBody.new(), "Pendulum")
	body.position = Vector3(0, 2, 0)
	body.rotation_degrees = Vector3(0, 0, swing_deg)
	var hinge: MujocoJoint = add(body, MujocoJoint.new(), "Hinge")
	hinge.rotation_degrees = Vector3(90, 0, 0) # axis (local Y) along Godot +Z
	var bob: MujocoGeom = add(body, MujocoGeom.new(), "Bob")
	bob.shape = MujocoGeom.SHAPE_SPHERE
	bob.radius = 0.02
	bob.position = Vector3(0, -1, 0)
	return body


func run() -> void:
	# pre_step fires once per MuJoCo step: 500 times per simulated second at the default 2 ms.
	var world := make_world("StepWorld", true)
	var box := make_free_box(world, 2.0)
	world.pre_step.connect(_count_pre_step)
	await process_frame
	pre_steps = 0
	var start := world.get_simulation_time()
	await run_until(world, 1.0)
	var steps_expected := roundi((world.get_simulation_time() - start) / world.option_timestep)
	check("pre_step once per step (%d signals, %d steps)" % [pre_steps, steps_expected], pre_steps == steps_expected)

	# reset() returns to the scene's state.
	check("box has fallen", box.global_position.y < 1.0)
	world.reset()
	check_near("reset: time back to 0", world.get_simulation_time(), 0.0, 1e-12)
	check_near("reset: box back at its start", box.global_position.y, 2.0, 1e-4)

	# step() runs exactly the requested steps while paused, emitting pre_step for each.
	world.running = false
	pre_steps = 0
	world.step(10)
	check_near("step(10) advances 10 timesteps", world.get_simulation_time(), 10 * world.option_timestep, 1e-12)
	check("step(10) emits pre_step 10 times", pre_steps == 10)
	check("step() moves the bodies", box.global_position.y < 2.0)
	world.queue_free()

	# Hinge position is measured from the scene pose, in radians, right-hand rule about the axis.
	var w1 := make_world("SwingWorld")
	w1.running = false # check the pose before any step
	var swing := make_pendulum(w1, 10.0)
	var hinge: MujocoJoint = swing.get_node("Hinge")
	await process_frame
	check_near("scene pose is joint position 0", hinge.get_joint_position(), 0.0, 1e-9)
	w1.running = true
	var lowest := 0.0
	while w1.get_simulation_time() < 2.0:
		await physics_frame
		lowest = minf(lowest, hinge.get_joint_position())
	# Released at +10 deg about +Z, it swings through the bottom to the other side: -20 deg relative.
	check_near("swings to -20 deg relative to the scene pose", lowest, deg_to_rad(-20.0), deg_to_rad(0.5))
	w1.queue_free()

	# set_joint_position moves the body right away.
	var w2 := make_world("SetWorld")
	var hanging := make_pendulum(w2, 0.0)
	var hinge2: MujocoJoint = hanging.get_node("Hinge")
	var bob: Node3D = hanging.get_node("Bob")
	await process_frame
	w2.running = false
	hinge2.set_joint_position(deg_to_rad(10.0))
	check_near("joint position reads back", hinge2.get_joint_position(), deg_to_rad(10.0), 1e-9)
	check_near("bob moved to +x sin(10 deg)", bob.global_position.x, sin(deg_to_rad(10.0)), 1e-4)

	# A pre_step controller runs before every step, so it can hold the joint exactly.
	w2.running = true
	var hold := func() -> void:
		hinge2.set_joint_position(0.3)
		hinge2.set_joint_velocity(0.0)
	w2.pre_step.connect(hold)
	await run_until(w2, w2.get_simulation_time() + 0.5)
	check_near("pre_step controller holds 0.3 rad", hinge2.get_joint_position(), 0.3, 1e-3)
	w2.pre_step.disconnect(hold)
	w2.queue_free()

	# Limits are in radians.
	var w3 := make_world("LimitWorld")
	var limited := make_pendulum(w3, 0.0)
	var hinge3: MujocoJoint = limited.get_node("Hinge")
	hinge3.limited = true
	hinge3.range = Vector2(-0.2, 0.2)
	await process_frame
	hinge3.set_joint_velocity(5.0)
	var widest := 0.0
	while w3.get_simulation_time() < 1.0:
		await physics_frame
		widest = maxf(widest, absf(hinge3.get_joint_position()))
	# Without the limit, 5 rad/s would swing the bob to about 106 degrees. MuJoCo's limits are
	# soft (0.02 s default time constant), so a fast hit overshoots a little (~0.035 rad here).
	check("swing reaches the 0.2 rad limit (max %.3f)" % widest, widest > 0.17)
	check("the limit stops the swing (max %.3f)" % widest, widest < 0.3)
	w3.queue_free()

	# Slide position: along the node's local Y (up), so a free fall is negative.
	var w4 := make_world("SlideWorld")
	var slider: MujocoBody = add(w4, MujocoBody.new(), "Slider")
	slider.position = Vector3(0, 5, 0)
	var slide: MujocoJoint = add(slider, MujocoJoint.new(), "Slide")
	slide.type = MujocoJoint.TYPE_SLIDE
	add(slider, MujocoGeom.new(), "Geom")
	await run_until(w4, 0.5)
	var t := w4.get_simulation_time()
	check_near("slide position = -g t^2 / 2", slide.get_joint_position(), -0.5 * G * t * t, 0.01)
	check_near("slide velocity = -g t", slide.get_joint_velocity(), -G * t, 0.02)

	# Ball and free joints have no single value.
	var free_joint: MujocoJoint = MujocoJoint.new()
	free_joint.type = MujocoJoint.TYPE_FREE
	print("(the next error is expected)")
	check("free joint position is unavailable", free_joint.get_joint_position() == 0.0)
	free_joint.free()
