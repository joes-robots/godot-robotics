extends "res://tests/test_base.gd"
## Sites and sensors: readings in Godot's conventions (Y-up, global or site-local frames).


func vectors_near(label: String, actual: Vector3, expected: Vector3, tolerance: float) -> void:
	var ok := actual.distance_to(expected) <= tolerance
	print("%s %s: got %s, expected %s (+/- %s)" % ["PASS" if ok else "FAIL", label, actual, expected, tolerance])
	if not ok:
		failures += 1


func bases_near(label: String, actual: Basis, expected: Basis, tolerance: float) -> void:
	var error := (actual.x - expected.x).length() + (actual.y - expected.y).length() + (actual.z - expected.z).length()
	var ok := error <= tolerance
	print("%s %s: basis difference %.7f (max %.7f)" % ["PASS" if ok else "FAIL", label, error, tolerance])
	if not ok:
		failures += 1


func add_site(parent: Node, site_name: String, xform := Transform3D()) -> MujocoSite:
	var site: MujocoSite = add(parent, MujocoSite.new(), site_name)
	site.transform = xform
	return site


## A sensor of the given type under p_target (so it measures p_target by default).
func add_sensor(p_target: Node, sensor_name: String, type: MujocoSensor.Type) -> MujocoSensor:
	var sensor: MujocoSensor = add(p_target, MujocoSensor.new(), sensor_name)
	sensor.type = type
	return sensor


func run() -> void:
	# MuJoCo treats bodies welded to the world (no joints) as static: their
	# acceleration sensors read 0, not gravity's reaction.
	var fw := make_world("FixedWorld")
	var fixed: MujocoBody = add(fw, MujocoBody.new(), "Fixed")
	add(fixed, MujocoGeom.new(), "Geom")
	var welded := add_sensor(add_site(fixed, "Site"), "Accel", MujocoSensor.TYPE_ACCELEROMETER)
	await process_frame
	vectors_near("static body: accelerometer reads 0 (MuJoCo convention)", welded.get_value(), Vector3.ZERO, 1e-9)
	fw.queue_free()

	# Frame sensors are global, even in a moved and rotated world; a body is measured at its node.
	var tilted := make_world("TiltedWorld")
	tilted.position = Vector3(10, 0, 0)
	tilted.rotation_degrees = Vector3(0, 0, 90)
	var faller := make_free_box(tilted, 0.0)
	var corner := add_site(faller, "Corner", Transform3D(Basis(Vector3.UP, 0.3), Vector3(0.3, 0.2, 0)))
	var body_pos := add_sensor(faller, "Pos", MujocoSensor.TYPE_FRAME_POSITION)
	var body_quat := add_sensor(faller, "Quat", MujocoSensor.TYPE_FRAME_ORIENTATION)
	var body_linvel := add_sensor(faller, "LinVel", MujocoSensor.TYPE_FRAME_LINEAR_VELOCITY)
	var body_angvel := add_sensor(faller, "AngVel", MujocoSensor.TYPE_FRAME_ANGULAR_VELOCITY)
	var site_pos := add_sensor(corner, "Pos", MujocoSensor.TYPE_FRAME_POSITION)
	var site_quat := add_sensor(corner, "Quat", MujocoSensor.TYPE_FRAME_ORIENTATION)
	var falling_accel := add_sensor(corner, "Accel", MujocoSensor.TYPE_ACCELEROMETER)
	await process_frame
	faller.set_angular_velocity(Vector3(0.5, 0, 0))
	await run_until(tilted, 0.5)
	vectors_near("body position", body_pos.get_value(), faller.global_position, 1e-5)
	bases_near("body orientation", Basis(body_quat.get_value() as Quaternion), faller.global_basis, 1e-5)
	vectors_near("body linear velocity", body_linvel.get_value(), faller.get_linear_velocity(), 1e-5)
	vectors_near("body angular velocity", body_angvel.get_value(), faller.get_angular_velocity(), 1e-5)
	vectors_near("site position", site_pos.get_value(), corner.global_position, 1e-5)
	bases_near("site orientation", Basis(site_quat.get_value() as Quaternion), corner.global_basis, 1e-5)
	# Free fall: no proper acceleration (the spin adds a little centripetal acceleration).
	vectors_near("free fall: accelerometer reads ~0", falling_accel.get_value(), Vector3.ZERO, 0.2)
	tilted.queue_free()

	# Gyro and velocimeter, in the site's frame. Site turned +90 degrees about X: its
	# local Z points down. Site turned +90 degrees about Y: its local Z points along +X.
	var zw := make_world("SpinWorld")
	zw.option_gravity = Vector3.ZERO
	var spinner := make_free_box(zw, 0.0)
	var gyro := add_sensor(add_site(spinner, "ZDown", Transform3D(Basis(Vector3.RIGHT, PI / 2), Vector3.ZERO)), "Gyro", MujocoSensor.TYPE_GYRO)
	var velocimeter := add_sensor(add_site(spinner, "ZForward", Transform3D(Basis(Vector3.UP, PI / 2), Vector3.ZERO)), "Vel", MujocoSensor.TYPE_VELOCIMETER)
	await process_frame
	spinner.set_angular_velocity(Vector3(0, 2, 0))
	spinner.set_linear_velocity(Vector3(1, 0, 0))
	vectors_near("gyro: 2 rad/s about global Y is -2 about the site's Z", gyro.get_value(), Vector3(0, 0, -2), 1e-6)
	vectors_near("velocimeter: global +X is the site's +Z", velocimeter.get_value(), Vector3(0, 0, 1), 1e-6)
	await run_until(zw, 0.5)
	vectors_near("gyro keeps reading while turning", gyro.get_value(), Vector3(0, 0, -2), 1e-6)
	zw.queue_free()

	# A box resting on the floor: the touch site under it carries its whole weight.
	var rw := make_world("RestWorld", true)
	var resting := make_free_box(rw, 0.5)
	var pad := add_site(resting, "Pad", Transform3D(Basis(), Vector3(0, -0.5, 0)))
	pad.shape = MujocoGeom.SHAPE_BOX
	pad.size = Vector3(1.1, 0.2, 1.1)
	var touch := add_sensor(pad, "Touch", MujocoSensor.TYPE_TOUCH)
	var rest_accel := add_sensor(pad, "Accel", MujocoSensor.TYPE_ACCELEROMETER)
	var top := add_site(resting, "Top", Transform3D(Basis(), Vector3(0, 0.5, 0)))
	var no_touch := add_sensor(top, "Touch", MujocoSensor.TYPE_TOUCH)
	# Rotated +90 degrees about Z, the site's local +X points up.
	var turned := add_sensor(add_site(resting, "Turned", Transform3D(Basis(Vector3.BACK, PI / 2), Vector3.ZERO)), "Accel", MujocoSensor.TYPE_ACCELEROMETER)
	await run_until(rw, 1.5)
	check_near("touch reads the weight m g", touch.get_value(), 1000.0 * G, 1000.0 * G * 0.01)
	check_near("a site away from the contacts reads 0", no_touch.get_value(), 0.0, 1e-9)
	vectors_near("accelerometer at rest reads (0, g, 0)", rest_accel.get_value(), Vector3(0, G, 0), 0.05)
	vectors_near("in the site's frame: a rotated site reads (g, 0, 0)", turned.get_value(), Vector3(G, 0, 0), 0.05)

	# The model has the site (with its shape) and the sensors, referring to their targets.
	var elements := mjcf_elements(rw.get_mjcf())
	check("MJCF has the site", elements.get("Box/Pad", {}).get("type", "") == "box")
	check_numbers("site size is MuJoCo half-sizes", elements.get("Box/Pad", {}).get("size", ""), "0.55 0.55 0.1")
	check("touch sensor on its site", elements.get("Box/Pad/Touch", {}).get("site", "") == "Box/Pad")
	rw.queue_free()

	# Joint and actuator sensors agree with the joint's and actuator's own getters.
	var jw := make_world("JointWorld")
	jw.option_gravity = Vector3.ZERO
	var rotor: MujocoBody = add(jw, MujocoBody.new(), "Rotor")
	var hinge: MujocoJoint = add(rotor, MujocoJoint.new(), "Hinge")
	add(rotor, MujocoGeom.new(), "Geom")
	var motor: MujocoActuator = add(hinge, MujocoActuator.new(), "Motor")
	motor.ctrl = 50.0
	var joint_pos := add_sensor(hinge, "Pos", MujocoSensor.TYPE_JOINT_POSITION)
	var joint_vel := add_sensor(hinge, "Vel", MujocoSensor.TYPE_JOINT_VELOCITY)
	var force := add_sensor(motor, "Force", MujocoSensor.TYPE_ACTUATOR_FORCE)
	await run_until(jw, 0.5)
	check("joint position moved (%.4f)" % hinge.get_joint_position(), hinge.get_joint_position() > 0.01)
	check_near("joint position sensor", joint_pos.get_value(), hinge.get_joint_position(), 1e-12)
	check_near("joint velocity sensor", joint_vel.get_value(), hinge.get_joint_velocity(), 1e-12)
	check_near("actuator force sensor", force.get_value(), 50.0, 1e-12)

	# A sensor whose target doesn't fit its type is left out; the model still compiles.
	var stray := add_sensor(rotor, "Stray", MujocoSensor.TYPE_ACCELEROMETER)
	print("(the next warning and error are expected)")
	elements = mjcf_elements(jw.get_mjcf())
	check("a sensor without a site is left out", not elements.has("Rotor/Stray") and elements.has("Rotor/Hinge/Pos"))
	check("its value is null", stray.get_value() == null)
	jw.queue_free()
