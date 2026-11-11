extends "res://tests/test_base.gd"
## Actuators: motor, position and velocity types, gear, control and force limits.
## The rotor is a 0.1 m sphere (density 1000 kg/m^3) on a hinge through its center,
## in zero gravity: mass 4/3 pi r^3 rho, moment of inertia 2/5 m r^2.

const RADIUS := 0.1
const ROTOR_MASS := 4.0 / 3.0 * PI * RADIUS * RADIUS * RADIUS * 1000.0
const ROTOR_INERTIA := 0.4 * ROTOR_MASS * RADIUS * RADIUS


## A world without gravity holding a rotor; returns its hinge.
func make_rotor(world_name: String) -> MujocoJoint:
	var world := make_world(world_name)
	world.option_gravity = Vector3.ZERO
	var body: MujocoBody = add(world, MujocoBody.new(), "Rotor")
	var hinge: MujocoJoint = add(body, MujocoJoint.new(), "Hinge")
	var geom: MujocoGeom = add(body, MujocoGeom.new(), "Geom")
	geom.shape = MujocoGeom.SHAPE_SPHERE
	geom.radius = RADIUS
	return hinge


## An actuator under p_joint (so it drives that joint by default).
func add_actuator(p_joint: MujocoJoint, type: MujocoActuator.Type) -> MujocoActuator:
	var actuator: MujocoActuator = add(p_joint, MujocoActuator.new(), "Actuator")
	actuator.type = type
	return actuator


func run() -> void:
	# Motor: angular acceleration = gear * ctrl / I.
	var hinge := make_rotor("MotorWorld")
	var world: MujocoWorld = hinge.get_parent().get_parent()
	var motor := add_actuator(hinge, MujocoActuator.TYPE_MOTOR)
	motor.ctrl = ROTOR_INERTIA
	await run_until(world, 1.0)
	var t := world.get_simulation_time()
	check_near("motor: w = ctrl t / I", hinge.get_joint_velocity(), t, 1e-6)
	check_near("motor force reads ctrl", motor.get_force(), ROTOR_INERTIA, 1e-12)

	# ctrl survives reset() ...
	world.reset()
	await run_until(world, 0.5)
	check_near("ctrl kept across reset()", hinge.get_joint_velocity(), world.get_simulation_time(), 1e-6)

	# ... and a recompile, here from changing gear: the joint gets gear times the force.
	motor.gear = 2.0
	motor.ctrl = ROTOR_INERTIA / 2.0
	await process_frame
	check_near("recompile reset the simulation", world.get_simulation_time(), 0.0, 0.1)
	t = world.get_simulation_time()
	await run_until(world, t + 0.5)
	check_near("gear 2 doubles the torque", hinge.get_joint_velocity(), world.get_simulation_time(), 1e-6)
	check_near("force is before gear", motor.get_force(), ROTOR_INERTIA / 2.0, 1e-12)

	# Force limits clamp the actuator force.
	motor.gear = 1.0
	motor.ctrl = ROTOR_INERTIA
	motor.force_limited = true
	motor.force_range = Vector2(-0.25, 0.25) * ROTOR_INERTIA
	await process_frame
	await run_until(world, 1.0)
	check_near("force limit: a quarter of the torque", hinge.get_joint_velocity(), 0.25 * world.get_simulation_time(), 1e-6)
	check_near("force reads the limit", motor.get_force(), 0.25 * ROTOR_INERTIA, 1e-9) # Vector2 is 32-bit
	world.queue_free()

	# Position servo, critically damped, settles on its target (radians, from the scene pose).
	hinge = make_rotor("PositionWorld")
	world = hinge.get_parent().get_parent()
	var servo := add_actuator(hinge, MujocoActuator.TYPE_POSITION)
	servo.kp = 1.0
	servo.kv = 2.0 * sqrt(servo.kp * ROTOR_INERTIA)
	servo.ctrl = 1.0
	await run_until(world, 2.0)
	check_near("position servo reaches 1 rad", hinge.get_joint_position(), 1.0, 1e-3)
	check_near("and stops", hinge.get_joint_velocity(), 0.0, 1e-3)
	servo.ctrl = -0.5
	await run_until(world, 4.0)
	check_near("follows a new target", hinge.get_joint_position(), -0.5, 1e-3)

	# Control limits clamp the target.
	servo.ctrl_limited = true
	servo.ctrl_range = Vector2(-0.3, 0.3)
	servo.ctrl = 1.0
	await process_frame
	await run_until(world, 2.0)
	check_near("control limit clamps the target to 0.3 rad", hinge.get_joint_position(), 0.3, 1e-3)
	check("get_ctrl is not clamped", servo.ctrl == 1.0)
	world.queue_free()

	# Velocity servo reaches its target velocity; with a force limit it only accelerates slowly.
	hinge = make_rotor("VelocityWorld")
	world = hinge.get_parent().get_parent()
	var velocity := add_actuator(hinge, MujocoActuator.TYPE_VELOCITY)
	velocity.kv = 1.0
	velocity.ctrl = 2.0
	await run_until(world, 0.5)
	check_near("velocity servo reaches 2 rad/s", hinge.get_joint_velocity(), 2.0, 1e-3)
	velocity.force_limited = true
	velocity.force_range = Vector2(-0.5, 0.5) * ROTOR_INERTIA
	await process_frame
	await run_until(world, 1.0)
	check_near("force-limited: a = 0.5 rad/s^2", hinge.get_joint_velocity(), 0.5 * world.get_simulation_time(), 1e-6)
	world.queue_free()

	# A position servo against gravity on a slide joint settles m g / kp below its target.
	var gw := make_world("SlideWorld")
	var box: MujocoBody = add(gw, MujocoBody.new(), "Box")
	box.position = Vector3(0, 3, 0)
	var slide: MujocoJoint = add(box, MujocoJoint.new(), "Slide")
	slide.type = MujocoJoint.TYPE_SLIDE
	add(box, MujocoGeom.new(), "Geom") # 1 m box: 1000 kg
	var lift := add_actuator(slide, MujocoActuator.TYPE_POSITION)
	lift.kp = 1e5
	lift.kv = 2.0 * sqrt(lift.kp * 1000.0)
	lift.ctrl = 0.5
	await run_until(gw, 2.0)
	check_near("slide servo sags by m g / kp", slide.get_joint_position(), 0.5 - 1000.0 * G / lift.kp, 1e-4)
	check_near("holding the weight", lift.get_force(), 1000.0 * G, 0.1)
	check_near("the body moved up", box.global_position.y, 3.0 + slide.get_joint_position(), 1e-6)

	# The actuator joins the MJCF, targeting its joint by name; an actuator on a separate
	# joint path works too.
	var remote: MujocoActuator = add(gw, MujocoActuator.new(), "Remote")
	remote.target = NodePath("../Box/Slide")
	var elements := mjcf_elements(gw.get_mjcf())
	check("MJCF has the actuator", elements.has("Box/Slide/Actuator"))
	check("targeting its joint", elements.get("Box/Slide/Actuator", {}).get("joint", "") == "Box/Slide")
	check("a joint path elsewhere in the tree works", elements.get("Remote", {}).get("joint", "") == "Box/Slide")

	# An actuator without a joint is left out; the rest of the model still compiles.
	var stray: MujocoActuator = add(box, MujocoActuator.new(), "Stray")
	print("(the next warning and error are expected)")
	elements = mjcf_elements(gw.get_mjcf())
	check("a stray actuator is left out", not elements.has("Box/Stray") and elements.has("Remote"))
	check("its force is unavailable", stray.get_force() == 0.0)
	gw.queue_free()
