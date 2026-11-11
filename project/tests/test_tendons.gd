extends "res://tests/test_base.gd"
## Tendons (fixed and spatial, wrapping, pulleys, limits, actuators, sensors,
## classes) and joint limit settings, built in Godot.


## A body at p_position with a 0.2 m box and a slide joint along Godot's X axis.
func slider(world: MujocoWorld, body_name: String, p_position: Vector3) -> MujocoJoint:
	var body: MujocoBody = add(world, MujocoBody.new(), body_name)
	body.position = p_position
	var joint: MujocoJoint = add(body, MujocoJoint.new(), "Slide")
	joint.type = MujocoJoint.TYPE_SLIDE
	# The joint axis is the node's local Y; turned -90 degrees about Z, that's +X.
	joint.rotation_degrees = Vector3(0, 0, -90)
	var geom: MujocoGeom = add(body, MujocoGeom.new(), "Geom")
	geom.size = Vector3(0.2, 0.2, 0.2)
	return joint


func path_step(tendon: MujocoTendon, target: Node, coef := 1.0) -> MujocoTendonWrap:
	var step: MujocoTendonWrap = add(tendon, MujocoTendonWrap.new(), "Wrap")
	if target:
		step.target = step.get_path_to(target)
	step.coef = coef
	return step


func static_site(world: MujocoWorld, site_name: String, p_position: Vector3) -> MujocoSite:
	var site: MujocoSite = add(world, MujocoSite.new(), site_name)
	site.position = p_position
	return site


func drawn_segments(tendon: MujocoTendon) -> MultiMesh:
	return (tendon.get_child(0, true) as MultiMeshInstance3D).multimesh


func run() -> void:
	# A fixed tendon couples joints: length = qA - qB, limited to 0.1. Pushing A
	# drags B along once the tendon reaches its limit.
	var fw := make_world("FixedWorld")
	fw.option_gravity = Vector3.ZERO
	var a := slider(fw, "A", Vector3(0, 0, 0))
	var b := slider(fw, "B", Vector3(0, 2, 0))
	var coupling: MujocoTendon = add(fw, MujocoTendon.new(), "Coupling")
	path_step(coupling, a, 1.0)
	path_step(coupling, b, -1.0)
	coupling.limited = true
	coupling.range = Vector2(-0.1, 0.1)
	var motor: MujocoActuator = add(a, MujocoActuator.new(), "Motor")
	motor.ctrl = 20.0
	var length_sensor: MujocoSensor = add(coupling, MujocoSensor.new(), "Length")
	length_sensor.type = MujocoSensor.TYPE_TENDON_POSITION
	await run_until(fw, 1.0)
	var qa := a.get_joint_position()
	var qb := b.get_joint_position()
	check("the motor moves A (%.3f)" % qa, qa > 0.3)
	check("the tendon drags B along (%.3f)" % qb, qb > 0.2)
	check_near("held at the tendon's limit", qa - qb, 0.1, 0.01)
	check_near("length = sum of coefficient * position", coupling.get_length(), qa - qb, 1e-9)
	check_near("tendon position sensor", length_sensor.get_value(), coupling.get_length(), 1e-9)
	var elements := mjcf_elements(fw.get_mjcf())
	check_numbers("a fixed tendon in the MJCF", elements.get("Coupling", {}).get("range", "0"), "-0.1 0.1")
	fw.queue_free()

	# An actuator under a tendon drives it: a position servo on a fixed tendon with
	# coefficient 2 holds the tendon at ctrl, so the joint at ctrl / 2.
	var aw := make_world("ActuatedWorld")
	aw.option_gravity = Vector3.ZERO
	var c := slider(aw, "C", Vector3.ZERO)
	var driven: MujocoTendon = add(aw, MujocoTendon.new(), "Driven")
	path_step(driven, c, 2.0)
	var servo: MujocoActuator = add(driven, MujocoActuator.new(), "Servo")
	servo.type = MujocoActuator.TYPE_POSITION
	servo.kp = 200.0
	servo.kv = 40.0
	servo.ctrl = 0.6
	await run_until(aw, 3.0)
	check_near("the servo sets the tendon length", driven.get_length(), 0.6, 0.01)
	check_near("so the joint moves half as far", c.get_joint_position(), 0.3, 0.005)
	check("it drives the tendon", mjcf_elements(aw.get_mjcf()).get("Driven/Servo", {}).get("tendon", "") == "Driven")
	# The old name of `target` still works (scenes saved before tendons use it).
	servo.set("joint", NodePath("../../C/Slide"))
	check("`joint` sets `target`", servo.target == NodePath("../../C/Slide"))
	aw.queue_free()

	# A spatial tendon as a rope: a weight hangs from a fixed site on a limited
	# tendon, 1.5 m long.
	var rw := make_world("RopeWorld")
	var anchor := static_site(rw, "Anchor", Vector3(0, 3, 0))
	var weight: MujocoBody = add(rw, MujocoBody.new(), "Weight")
	weight.position = Vector3(0.3, 2, 0)
	var free: MujocoJoint = add(weight, MujocoJoint.new(), "Free")
	free.type = MujocoJoint.TYPE_FREE
	var ball: MujocoGeom = add(weight, MujocoGeom.new(), "Ball")
	ball.shape = MujocoGeom.SHAPE_SPHERE
	ball.radius = 0.1
	var hook: MujocoSite = add(weight, MujocoSite.new(), "Hook")
	var rope: MujocoTendon = add(rw, MujocoTendon.new(), "Rope")
	path_step(rope, anchor)
	path_step(rope, hook)
	rope.limited = true
	rope.range = Vector2(0, 1.5)
	rope.width = 0.01
	await run_until(rw, 4.0)
	check_near("the rope holds the weight at its length", anchor.global_position.distance_to(hook.global_position), 1.5, 0.02)
	check_near("and its length is the distance", rope.get_length(), anchor.global_position.distance_to(hook.global_position), 1e-4)
	# (The headless test renderer keeps instance counts but not transforms.)
	check("drawn as one segment", drawn_segments(rope).instance_count == 1)
	rw.queue_free()

	# Wrapping: from (-1, 1, 0) to (1, 1, 0) over a sphere of radius 0.5 at (0, 1, 0)
	# (the side site above picks the top). Two tangents of sqrt(1 - 0.25) and a
	# 60 degree arc.
	var ww := make_world("WrapWorld")
	var left := static_site(ww, "Left", Vector3(-1, 1, 0))
	var right := static_site(ww, "Right", Vector3(1, 1, 0))
	var top := static_site(ww, "Top", Vector3(0, 2, 0))
	var pulley_body: MujocoGeom = add(ww, MujocoGeom.new(), "Drum")
	pulley_body.shape = MujocoGeom.SHAPE_SPHERE
	pulley_body.radius = 0.5
	pulley_body.position = Vector3(0, 1, 0)
	var over: MujocoTendon = add(ww, MujocoTendon.new(), "Over")
	path_step(over, left)
	var around := path_step(over, pulley_body)
	around.side_site = around.get_path_to(top)
	path_step(over, right)
	# A pulley: the branch after it counts 1 / divisor.
	var split: MujocoTendon = add(ww, MujocoTendon.new(), "Split")
	path_step(split, left)
	path_step(split, top)
	var pulley := path_step(split, null)
	pulley.divisor = 2.0
	path_step(split, top)
	path_step(split, right)
	await process_frame
	check_near("wraps over the sphere", over.get_length(), 2.0 * sqrt(0.75) + 0.5 * PI / 3.0, 1e-4)
	check("drawn around it", drawn_segments(over).instance_count >= 3)
	check_near("a pulley halves its second branch", split.get_length(), sqrt(2.0) + sqrt(2.0) / 2.0, 1e-4)
	ww.queue_free()

	# Joint limits: a softer limit (larger time constant) lets a weight on a vertical
	# slide sink further past its lower limit.
	var lw := make_world("LimitWorld")
	var depths := []
	for timeconst in [0.02, 0.1]:
		var body: MujocoBody = add(lw, MujocoBody.new(), "Limit%d" % depths.size())
		body.position = Vector3(depths.size() * 2.0, 1, 0)
		var joint: MujocoJoint = add(body, MujocoJoint.new(), "Slide")
		joint.type = MujocoJoint.TYPE_SLIDE
		joint.limited = true
		joint.range = Vector2(-0.1, 0.1)
		joint.limit_solref_timeconst = timeconst
		add(body, MujocoGeom.new(), "Geom")
		depths.append(joint)
	await run_until(lw, 2.0)
	var stiff: float = -0.1 - depths[0].get_joint_position()
	var soft: float = -0.1 - depths[1].get_joint_position()
	check("a softer limit gives more (%.4f m vs %.4f m)" % [soft, stiff], soft > 3.0 * stiff and stiff > 0.0)
	check_numbers("solreflimit in the MJCF (MuJoCo leaves out the default dampratio)", mjcf_elements(lw.get_mjcf()).get("Limit1/Slide", {}).get("solreflimit", "0"), "0.1")
	lw.queue_free()

	# Classes have a tendon section.
	var cw := make_world("ClassWorld")
	var main := MujocoDefault.new()
	main.name = "main"
	cw.add_child(main)
	main.set("tendon/width", 0.02)
	main.set("joint/limit_margin", 0.01)
	var d := slider(cw, "D", Vector3.ZERO)
	var classed: MujocoTendon = add(cw, MujocoTendon.new(), "Classed")
	path_step(classed, d)
	await process_frame
	check_near("a tendon inherits its class's width", classed.width, 0.02, 1e-12)
	check("without overriding it", classed.overrides.is_empty())
	check_near("a joint inherits its class's limit margin", d.limit_margin, 0.01, 1e-12)

	# A tendon whose path is wrong is left out, and the rest still compiles.
	var mixed: MujocoTendon = add(cw, MujocoTendon.new(), "Mixed")
	path_step(mixed, d)
	path_step(mixed, static_site(cw, "Site", Vector3.ZERO))
	await process_frame
	var class_elements := mjcf_elements(cw.get_mjcf())
	check("a mixed path is left out", not class_elements.has("Mixed") and class_elements.has("Classed"))
	cw.queue_free()

	# Import: tendons from a file, with their classes, paths, actuators and sensors.
	var importer := MujocoImporter.new()
	var world := importer.import_file("res://tests/models/tendon.xml")
	check("tendon.xml imports", world != null)
	if world == null:
		return
	check("nothing left out (report: %s)" % [importer.get_report()], importer.get_report().is_empty())
	var diffs := importer.verify(world, 1000)
	check("faithful: same compiled values and 1000 identical steps %s" % [diffs], diffs.is_empty())
	var couple: MujocoTendon = world.get_node("couple")
	check("a fixed tendon from its class", couple.default_class == &"limb" and couple.is_limited() and not couple.overrides.has("stiffness"))
	check_near("with the class's limit settings", couple.limit_solref_timeconst, 0.04, 1e-12)
	var steps := couple.get_children()
	check("its path: the joints in order, then its actuator and sensor", steps.size() == 4 and steps[0].get_node(steps[0].target) == world.get_node("arm/shoulder") and is_equal_approx(steps[1].coef, -0.5))
	check("an actuator under its tendon", steps[2] is MujocoActuator and (steps[2] as MujocoActuator).type == MujocoActuator.TYPE_POSITION)
	var cable: MujocoTendon = world.get_node("cable")
	var drum_step: MujocoTendonWrap = cable.get_node("drum")
	check("a wrapping geom with its side site", drum_step.get_node(drum_step.side_site) == world.get_node("side"))
	check_near("the main class's tendon width", cable.width, 0.005, 1e-12)
	check("a pulley", world.get_node("split/Pulley") is MujocoTendonWrap and is_equal_approx(world.get_node("split/Pulley").divisor, 2.0))
	check("tendon sensors under their tendon", (world.get_node("couple/couple_len") as MujocoSensor).type == MujocoSensor.TYPE_TENDON_POSITION)
	var elbow: MujocoJoint = world.get_node("arm/forearm/elbow")
	check("joint limit settings from the class", elbow.default_class == &"" and is_equal_approx(elbow.limit_margin, 0.01) and not elbow.overrides.has("limit_margin"))
	world.free()
