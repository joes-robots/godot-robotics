extends "res://tests/test_base.gd"
## MJCF import: a model with the supported features imports faithfully (verify()
## compares the compiled models and runs both simulations), classes are rebuilt and
## elements override only what differs; unsupported parts are reported.

const EXTRAS := """
<mujoco model="extras">
  <asset><hfield name="ground" nrow="2" ncol="2" size="1 1 0.1 0.1" elevation="0 0 0 0"/></asset>
  <worldbody>
    <geom type="hfield" hfield="ground"/>
    <body name="a" pos="0 0 1">
      <freejoint/>
      <inertial pos="0 0 0" mass="3" diaginertia="0.1 0.1 0.1"/>
      <geom name="rough" type="box" size="0.1 0.1 0.1" condim="1"/>
      <site name="s1"/>
    </body>
    <body name="b" pos="1 0 1">
      <joint name="h" axis="0 1 0" ref="10"/>
      <geom size="0.1"/>
      <site name="s2"/>
    </body>
  </worldbody>
  <tendon><spatial name="t"><site site="s1"/><site site="s2"/></spatial></tendon>
  <equality><connect body1="a" body2="b" anchor="0 0 0"/></equality>
  <actuator><muscle name="m" tendon="t"/></actuator>
  <sensor><jointpos joint="h" noise="0.1"/></sensor>
</mujoco>
"""


func report_has(report: PackedStringArray, text: String) -> bool:
	for line in report:
		if text in line:
			return true
	return false


func run() -> void:
	var importer := MujocoImporter.new()
	var world := importer.import_file("res://tests/models/arm.xml")
	check("arm.xml imports", world != null)
	if world == null:
		return
	root.add_child(world)
	await process_frame
	check("nothing left out (report: %s)" % [importer.get_report()], importer.get_report().is_empty())
	var diffs := importer.verify(world, 1000)
	check("faithful: same compiled values and 1000 identical steps %s" % [diffs], diffs.is_empty())

	# The world: name and options.
	check("world named after the model", world.name == "test_arm")
	check_near("options imported", world.option_timestep, 0.004, 1e-12)
	check("integrator imported", world.option_integrator == MujocoWorld.INTEGRATOR_IMPLICIT_FAST)

	# Classes: main mirrors MuJoCo's root class, with the nesting of the file.
	var main: MujocoDefault = world.get_node_or_null("main")
	check("main class", main != null and main.is_main())
	check_near("main has the file's joint damping", main.get("joint/damping"), 0.5, 1e-12)
	check("link class", world.get_node_or_null("link") is MujocoDefault)
	check("thin nested in link", world.get_node_or_null("link/thin") is MujocoDefault)
	check_near("class geom radius", world.get_node("link/thin").get("geom/radius"), 0.03, 1e-12)
	check_near("class servo kp", world.get_node("servo").get("actuator/kp"), 80.0, 1e-12)

	# Bodies and elements: names, childclass, class references, minimal overrides.
	var upper: MujocoBody = world.get_node("base/upper")
	check("childclass imported", upper.child_class == &"link")
	var lower_link: MujocoGeom = world.get_node("base/upper/lower/lower_link")
	check("geom class", lower_link.default_class == &"thin")
	check("geom inherits its shape and density", not lower_link.overrides.has("shape") and not lower_link.overrides.has("density"))
	check_near("fromto became the capsule length", lower_link.height, 0.4 + 2 * 0.03, 1e-6)
	var elbow: MujocoJoint = world.get_node("base/upper/lower/elbow")
	check_near("elbow damping overridden", elbow.overrides.get("damping", -1.0), 1.0, 1e-12)
	check("elbow armature inherited", not elbow.overrides.has("armature"))
	check_near("range converted to radians", elbow.range.y, deg_to_rad(120.0), 1e-6)
	var shoulder_servo: MujocoActuator = world.get_node("base/upper/shoulder/shoulder_servo")
	check("actuator under its joint, from its class", shoulder_servo.type == MujocoActuator.TYPE_POSITION and shoulder_servo.default_class == &"servo")
	check("class actuator overrides nothing", shoulder_servo.overrides.is_empty())
	var elbow_servo: MujocoActuator = world.get_node("base/upper/lower/elbow/elbow_servo")
	check("actuator overrides only kp", elbow_servo.overrides.keys() == ["kp"])
	check("motor imported", (world.get_node("slider/slide/slide_motor") as MujocoActuator).type == MujocoActuator.TYPE_MOTOR)
	check("sensor under its target", world.get_node_or_null("base/upper/lower/tip/tip_pos") is MujocoSensor)
	check("free joint", (world.get_node("ball/ball_free") as MujocoJoint).type == MujocoJoint.TYPE_FREE)

	# The imported world simulates: the ball falls onto the floor.
	await run_until(world, 1.5)
	check_near("the ball rests on the floor", world.get_node("ball").global_position.y, 0.1, 0.01)

	# It can be saved as a scene.
	var scene := PackedScene.new()
	check("packs into a scene", scene.pack(world) == OK)
	var copy: MujocoWorld = scene.instantiate()
	check("the saved scene has the same tree", copy.get_node_or_null("base/upper/lower/elbow/elbow_servo") != null)
	copy.free()
	world.queue_free()

	# Unsupported parts are left out and reported; the rest still imports.
	var extras := importer.import_string(EXTRAS)
	check("extras import", extras != null)
	var report := importer.get_report()
	print("report: %s" % [report])
	check("reports height field geoms", report_has(report, "Height field geoms"))
	# Imported since Phase 3.3 (see test_contact): explicit inertia, condim, joint ref.
	check("no longer reports inertia, contact parameters or ref", not report_has(report, "inertia") and not report_has(report, "condim") and not report_has(report, "ref"))
	check("reports tendons", report_has(report, "Tendons"))
	check("reports equality constraints", report_has(report, "Equality"))
	check("reports the tendon actuator", report_has(report, "Actuators other than"))
	check("reports sensor noise", report_has(report, "noise"))
	root.add_child(extras)
	await process_frame
	diffs = importer.verify(extras, 100)
	print("verify: %s" % [diffs])
	check("verify finds what was left out (the height field, the muscle)", report_has(diffs, "Geom '#0' was not imported") and report_has(diffs, "Actuator 'm' was not imported"))
	check("but not the inertia or contact parameters", not report_has(diffs, "mass") and not report_has(diffs, "condim"))
	extras.queue_free()

	# A missing file fails cleanly.
	print("(the next error is expected)")
	check("missing file returns null", importer.import_file("res://tests/models/missing.xml") == null)
	check("and says why", report_has(importer.get_report(), "Could not load"))
