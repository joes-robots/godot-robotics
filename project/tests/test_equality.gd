extends "res://tests/test_base.gd"
## Equality constraints (connect, weld, joint, tendon), switching them at runtime,
## their classes, and importing them.


## A free body at p_position with a 0.2 m sphere.
func ball(world: MujocoWorld, body_name: String, p_position: Vector3) -> MujocoBody:
	var body: MujocoBody = add(world, MujocoBody.new(), body_name)
	body.position = p_position
	var joint: MujocoJoint = add(body, MujocoJoint.new(), "Free")
	joint.type = MujocoJoint.TYPE_FREE
	var geom: MujocoGeom = add(body, MujocoGeom.new(), "Geom")
	geom.shape = MujocoGeom.SHAPE_SPHERE
	geom.radius = 0.1
	return body


## A body at p_position with a 0.2 m box and a slide joint along Godot's X axis.
func slider(world: MujocoWorld, body_name: String, p_position: Vector3) -> MujocoJoint:
	var body: MujocoBody = add(world, MujocoBody.new(), body_name)
	body.position = p_position
	var joint: MujocoJoint = add(body, MujocoJoint.new(), "Slide")
	joint.type = MujocoJoint.TYPE_SLIDE
	joint.rotation_degrees = Vector3(0, 0, -90)
	var geom: MujocoGeom = add(body, MujocoGeom.new(), "Geom")
	geom.size = Vector3(0.2, 0.2, 0.2)
	return joint


func run() -> void:
	# Connect: a ball joined to the world at a point 0.5 m beside it swings around
	# that point like a pendulum.
	var cw := make_world("ConnectWorld")
	var bob := ball(cw, "Bob", Vector3(0.5, 2, 0))
	var pivot: MujocoEquality = add(bob, MujocoEquality.new(), "Pivot")
	pivot.position = Vector3(-0.5, 0, 0)
	await run_until(cw, 1.0)
	check("the ball swings down (y = %.2f)" % bob.position.y, bob.position.y < 1.9)
	check_near("on a 0.5 m radius around the pivot", bob.position.distance_to(Vector3(0, 2, 0)), 0.5, 0.005)
	check_near("the anchor stays at the pivot", pivot.global_position.distance_to(Vector3(0, 2, 0)), 0.0, 0.005)
	var elements := mjcf_elements(cw.get_mjcf())
	check_numbers("a connect with its anchor in the MJCF", elements.get("Bob/Pivot", {}).get("anchor", "0"), "-0.5 0 0")
	cw.queue_free()

	# Weld: a ball welded to the world doesn't fall; two welded balls fall together.
	var ww := make_world("WeldWorld")
	var held := ball(ww, "Held", Vector3(0, 2, 0))
	var weld: MujocoEquality = add(held, MujocoEquality.new(), "Weld")
	weld.type = MujocoEquality.TYPE_WELD
	var a := ball(ww, "A", Vector3(2, 2, 0))
	var b := ball(ww, "B", Vector3(2.5, 2.2, 0))
	var pair: MujocoEquality = add(a, MujocoEquality.new(), "Pair")
	pair.type = MujocoEquality.TYPE_WELD
	pair.object2 = NodePath("../../B")
	await run_until(ww, 0.5)
	check_near("welded to the world, it stays", held.position.y, 2.0, 0.001)
	check("welded to each other, they fall (y = %.2f)" % a.position.y, a.position.y < 1.0)
	check_near("together", a.position.distance_to(b.position), Vector3(0.5, 0.2, 0).length(), 0.001)

	# Switching a constraint off at runtime: the held ball falls, and reset() keeps it off.
	weld.active = false
	await run_until(ww, 1.0)
	check("switched off, it falls (y = %.2f)" % held.position.y, held.position.y < 1.0)
	ww.reset()
	await run_until(ww, 0.3)
	check("still off after reset (y = %.2f)" % held.position.y, held.position.y < 1.8)
	weld.active = true
	ww.reset()
	await run_until(ww, 0.3)
	check_near("and on again", held.position.y, 2.0, 0.001)
	ww.queue_free()

	# Joint: q1 = 0.1 + 2 q2. A motor pushes joint 2; joint 1 follows.
	var jw := make_world("JointWorld")
	jw.option_gravity = Vector3.ZERO
	var j1 := slider(jw, "J1", Vector3(0, 0, 0))
	var j2 := slider(jw, "J2", Vector3(0, 2, 0))
	var coupling: MujocoEquality = add(j1, MujocoEquality.new(), "Coupling")
	coupling.type = MujocoEquality.TYPE_JOINT
	coupling.object2 = NodePath("../../../J2/Slide")
	coupling.poly0 = 0.1
	coupling.poly1 = 2.0
	var push: MujocoActuator = add(j2, MujocoActuator.new(), "Push")
	# Joint 2 carries joint 1 through the coupling: five times its own mass.
	push.ctrl = 40.0
	await run_until(jw, 1.0)
	var q1 := j1.get_joint_position()
	var q2 := j2.get_joint_position()
	check("joint 2 moves (%.3f)" % q2, q2 > 0.05)
	check_near("joint 1 follows the polynomial", q1, 0.1 + 2.0 * q2, 0.002)
	var polycoef: String = mjcf_elements(jw.get_mjcf()).get("J1/Slide/Coupling", {}).get("polycoef", "")
	check("polycoef in the MJCF (%s)" % polycoef, polycoef.begins_with("0.1 2"))
	jw.queue_free()

	# Tendon: length 1 = 0.5 * length 2, with one joint in each tendon.
	var tw := make_world("TendonWorld")
	tw.option_gravity = Vector3.ZERO
	var t1 := slider(tw, "T1", Vector3(0, 0, 0))
	var t2 := slider(tw, "T2", Vector3(0, 2, 0))
	var tendons: Array[MujocoTendon] = []
	for joint in [t1, t2]:
		var tendon: MujocoTendon = add(tw, MujocoTendon.new(), "Tendon%d" % (tendons.size() + 1))
		var step: MujocoTendonWrap = add(tendon, MujocoTendonWrap.new(), "Joint")
		step.target = step.get_path_to(joint)
		tendons.append(tendon)
	var lengths: MujocoEquality = add(tendons[0], MujocoEquality.new(), "Lengths")
	lengths.type = MujocoEquality.TYPE_TENDON
	lengths.object2 = NodePath("../../Tendon2")
	lengths.poly1 = 0.5
	var pull: MujocoActuator = add(t2, MujocoActuator.new(), "Pull")
	pull.ctrl = 2.0
	await run_until(tw, 1.0)
	check("tendon 2 lengthens (%.3f)" % tendons[1].get_length(), tendons[1].get_length() > 0.05)
	check_near("tendon 1 follows at half", tendons[0].get_length(), 0.5 * tendons[1].get_length(), 0.002)
	tw.queue_free()

	# Classes have an equality section; a constraint between the wrong kinds of
	# objects is left out and the rest still compiles.
	var dw := make_world("ClassWorld")
	var main := MujocoDefault.new()
	main.name = "main"
	dw.add_child(main)
	main.set("equality/solref_timeconst", 0.05)
	var c := ball(dw, "C", Vector3.ZERO)
	var classed: MujocoEquality = add(c, MujocoEquality.new(), "Classed")
	var wrong: MujocoEquality = add(c, MujocoEquality.new(), "Wrong")
	wrong.type = MujocoEquality.TYPE_JOINT
	await process_frame
	check_near("inherits its class's solref", classed.solref_timeconst, 0.05, 1e-12)
	var class_elements := mjcf_elements(dw.get_mjcf())
	check("a constraint on the wrong kind of object is left out", class_elements.has("C/Classed") and not class_elements.has("C/Wrong"))
	dw.queue_free()

	# Import: every kind, with classes, under their first object.
	var importer := MujocoImporter.new()
	var world := importer.import_file("res://tests/models/equality.xml")
	check("equality.xml imports", world != null)
	if world == null:
		return
	check("nothing left out (report: %s)" % [importer.get_report()], importer.get_report().is_empty())
	var diffs := importer.verify(world, 1000)
	check("faithful: same compiled values and 1000 identical steps %s" % [diffs], diffs.is_empty())
	var mimic: MujocoEquality = world.get_node("follower/j3/mimic")
	check("a joint coupling under joint 1, from its class", mimic.type == MujocoEquality.TYPE_JOINT and mimic.default_class == &"stiff" and mimic.overrides.is_empty())
	check("pointing at joint 2", mimic.get_node(mimic.object2) == world.get_node("base/j1"))
	check_near("with the class's solref", mimic.solref_timeconst, 0.002, 1e-12)
	var lock: MujocoEquality = world.get_node("base/link/j2/lock")
	check("a joint locked at poly0", lock.object2.is_empty() and is_equal_approx(lock.poly0, 0.2) and lock.poly1 == 0.0)
	var hang: MujocoEquality = world.get_node("hanging/hang")
	check("a connect to the world, anchored at its position", hang.object2.is_empty() and hang.position.is_equal_approx(Vector3(0, 0.3, 0)))
	var sites: MujocoEquality = world.get_node("base/s1/sites")
	check("a connect between sites", sites.get_node(sites.object2) == world.get_node("follower/s2"))
	var glue: MujocoEquality = world.get_node("welded/glue")
	check("a weld with its torque scale", glue.type == MujocoEquality.TYPE_WELD and is_equal_approx(glue.torque_scale, 2.0))
	check("an inactive tendon coupling", (world.get_node("t2/pair") as MujocoEquality).active == false)
	world.free()
