extends "res://tests/test_base.gd"
## Explicit body inertia, geom contact parameters, joint ref and springref, and
## contact exclusions: what they do in the simulation, and that they import.


func zero_g_world(world_name: String, with_floor := false) -> MujocoWorld:
	var world := make_world(world_name, with_floor)
	world.option_gravity = Vector3.ZERO
	return world


## Calls p_apply at the start of every physics frame for p_duration simulated
## seconds (as in test_bodies). Returns how long it acted.
func apply_each_frame(world: MujocoWorld, p_apply: Callable, p_duration: float) -> float:
	var t0 := world.get_simulation_time()
	physics_frame.connect(p_apply)
	await run_until(world, t0 + p_duration)
	physics_frame.disconnect(p_apply)
	await physics_frame
	return world.get_simulation_time() - t0


## Attributes of the first <tag> inside <body name="p_body">.
func body_child_attributes(xml: String, p_body: String, tag: String) -> Dictionary:
	var parser := XMLParser.new()
	parser.open_buffer(xml.to_utf8_buffer())
	var inside := false
	while parser.read() == OK:
		if parser.get_node_type() != XMLParser.NODE_ELEMENT:
			continue
		if parser.get_node_name() == "body":
			inside = parser.get_named_attribute_value_safe("name") == p_body
		elif inside and parser.get_node_name() == tag:
			var attributes := {}
			for i in parser.get_attribute_count():
				attributes[parser.get_attribute_name(i)] = parser.get_attribute_value(i)
			return attributes
	return {}


func run() -> void:
	# Explicit inertia replaces the geoms' mass: a 1 m box (1000 kg from its geom)
	# given 10 kg and moments (1, 2, 3) about its local x, y, z.
	var iw := zero_g_world("InertiaWorld")
	var box := make_free_box(iw, 0.0)
	box.explicit_inertia = true
	box.mass = 10.0
	box.inertia = Vector3(1, 2, 3)
	box.center_of_mass = Vector3(0, 0.5, 0)
	var inertial := body_child_attributes(iw.get_mjcf(), "Box", "inertial")
	check_numbers("inertial position in MuJoCo axes", inertial.get("pos", ""), "0 0 0.5")
	check_numbers("moments swap y and z like sizes", inertial.get("diaginertia", ""), "1 3 2")
	check_numbers("mass", inertial.get("mass", ""), "10")
	var elapsed: float = await apply_each_frame(iw, func() -> void:
		box.apply_central_force(Vector3(10, 0, 0))
		box.apply_torque(Vector3(0, 2, 0)), 1.0)
	check_near("a = F / m with the explicit mass", box.get_linear_velocity().x, elapsed, 0.02)
	check_near("alpha = T / I about local y", box.get_angular_velocity().y, elapsed, 0.03)
	iw.queue_free()

	# condim and priority: a frictionless box (condim 1, which wins by priority over
	# the floor's condim 3) keeps sliding; a normal one stops.
	var fw := make_world("FrictionWorld", true)
	var sticky := make_free_box(fw, 0.5)
	sticky.position.z = -2
	var slippery := make_free_box(fw, 0.5)
	slippery.name = "Slippery"
	slippery.position.z = 2
	var slippery_geom: MujocoGeom = slippery.get_node("Geom")
	slippery_geom.condim = 1
	slippery_geom.priority = 1
	await process_frame
	sticky.set_linear_velocity(Vector3(2, 0, 0))
	slippery.set_linear_velocity(Vector3(2, 0, 0))
	await run_until(fw, fw.get_simulation_time() + 1.0)
	check("condim 3 stops the box (x = %.3f)" % sticky.position.x, sticky.position.x < 0.4)
	check_near("condim 1 slides on without friction", slippery.position.x, 2.0, 0.05)
	var elements := mjcf_elements(fw.get_mjcf())
	check("condim in the MJCF", elements.get("Slippery/Geom", {}).get("condim", "") == "1")
	fw.queue_free()

	# solref: a softer contact (larger time constant) lets a resting box sink deeper.
	var sw := make_world("SoftWorld", true)
	var hard := make_free_box(sw, 0.5)
	hard.position.x = -2
	var soft := make_free_box(sw, 0.5)
	soft.name = "Soft"
	soft.position.x = 2
	(soft.get_node("Geom") as MujocoGeom).solref_timeconst = 0.1
	await run_until(sw, 2.0)
	var soft_depth := 0.5 - soft.position.y
	var hard_depth := 0.5 - hard.position.y
	check("soft contact sinks deeper (%.5f m vs %.5f m)" % [soft_depth, hard_depth], soft_depth > 3.0 * hard_depth and hard_depth > 0.0)
	sw.queue_free()

	# ref: the scene pose is joint position ref. springref: the spring rests there.
	var jw := zero_g_world("SpringWorld")
	var arm: MujocoBody = add(jw, MujocoBody.new(), "Arm")
	var hinge: MujocoJoint = add(arm, MujocoJoint.new(), "Hinge")
	hinge.rotation_degrees = Vector3(90, 0, 0)
	hinge.ref = 0.5
	hinge.springref = 0.8
	hinge.stiffness = 20.0
	hinge.damping = 18.0
	var bob: MujocoGeom = add(arm, MujocoGeom.new(), "Bob")
	bob.shape = MujocoGeom.SHAPE_SPHERE
	bob.radius = 0.1
	bob.position = Vector3(0, -1, 0)
	jw.running = false
	await process_frame
	check_near("the scene pose is position ref", hinge.get_joint_position(), 0.5, 1e-9)
	jw.running = true
	await run_until(jw, 4.0)
	check_near("the spring settles at springref", hinge.get_joint_position(), 0.8, 0.01)
	check_near("0.3 rad from the scene pose", arm.rotation.z, 0.3, 0.01)
	jw.queue_free()

	# Exclusions: overlapping boxes push apart unless excluded; a box excluded from
	# the world falls through the floor.
	for excluded in [false, true]:
		var ew := zero_g_world("ExcludeWorld")
		var a := make_free_box(ew, 0.0)
		var b := make_free_box(ew, 0.0)
		b.name = "B"
		b.position.x = 0.5
		if excluded:
			var exclude: MujocoContactExclude = add(a, MujocoContactExclude.new(), "Exclude")
			exclude.body2 = NodePath("../../B")
		await run_until(ew, 0.5)
		if excluded:
			check_near("excluded boxes stay put", b.position.x, 0.5, 1e-6)
			check("the exclusion is in the MJCF", mjcf_elements(ew.get_mjcf()).get("Box/Exclude", {}).get("body2", "") == "B")
		else:
			check("overlapping boxes push apart (x = %.3f)" % b.position.x, b.position.x > 0.6)
		ew.queue_free()
	var gw := make_world("GroundWorld", true)
	var sinker := make_free_box(gw, 0.5)
	var through: MujocoContactExclude = add(sinker, MujocoContactExclude.new(), "NoFloor")
	through.body2 = NodePath("../..")
	await run_until(gw, 1.0)
	check("excluded from the world, it falls through the floor (y = %.2f)" % sinker.position.y, sinker.position.y < -1.0)
	gw.queue_free()

	# All of it imports faithfully.
	var importer := MujocoImporter.new()
	var world := importer.import_file("res://tests/models/contact.xml")
	check("contact.xml imports", world != null)
	if world == null:
		return
	root.add_child(world)
	await process_frame
	check("nothing left out (report: %s)" % [importer.get_report()], importer.get_report().is_empty())
	var diffs := importer.verify(world, 1000)
	check("faithful: same compiled values and 1000 identical steps %s" % [diffs], diffs.is_empty())
	var base: MujocoBody = world.get_node("base")
	check("explicit inertia imported", base.explicit_inertia and is_equal_approx(base.mass, 4.0))
	var arm_geom: MujocoGeom = world.get_node("base/arm/arm_geom")
	check("contact parameters come from the classes", arm_geom.default_class == &"grippy" and arm_geom.condim == 6 and not arm_geom.overrides.has("condim"))
	check_near("solref from main", arm_geom.solref_timeconst, 0.01, 1e-12)
	check_near("hinge ref in radians", (world.get_node("base/arm/hinge") as MujocoJoint).ref, deg_to_rad(30.0), 1e-6)
	var no_tip: MujocoContactExclude = world.get_node_or_null("base/no_tip_base")
	check("exclusion under body1", no_tip != null and no_tip.get_node(no_tip.body2) == world.get_node("base/arm/tip"))
	check("exclusion with the world", world.get_node_or_null("ContactExclude") is MujocoContactExclude)
	world.queue_free()

	# An explicit mass of 0 (visual geoms) means no mass in MuJoCo; here that's
	# density 0, since our mass 0 means "from the density".
	var massless := importer.import_string("""
<mujoco>
  <default><default class="visual"><geom mass="0" contype="0" conaffinity="0"/></default></default>
  <worldbody>
    <body name="body" pos="0 0 1">
      <freejoint/>
      <geom class="visual" type="box" size="0.5 0.5 0.5"/>
      <geom name="collision" type="sphere" size="0.1" mass="2"/>
      <geom name="tiny" type="sphere" size="0.05" pos="0 0 0.2" mass="1e-6"/>
    </body>
  </worldbody>
</mujoco>
""")
	diffs = importer.verify(massless, 100)
	check("a geom with mass 0 adds no mass %s" % [diffs], diffs.is_empty())
	check_near("its class has density 0", massless.get_node("visual").get("geom/density"), 0.0, 1e-12)
	check("a tiny mass is kept, not taken for 0", is_equal_approx((massless.get_node("body/tiny") as MujocoGeom).mass, 1e-6))
	massless.free()
