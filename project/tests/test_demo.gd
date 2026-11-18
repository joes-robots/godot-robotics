extends "res://tests/test_base.gd"
## The demo scenes load, compile and behave as described.


func run() -> void:
	test_shapes()
	await test_workcell()


func test_workcell() -> void:
	var scene: PackedScene = load("res://demos/workcell.tscn")
	check("workcell scene loads", scene != null)
	var demo: Node3D = scene.instantiate()
	root.add_child(demo)
	var world: Node3D = demo.get_node("MujocoWorld")
	check("workcell compiles", world.is_compiled() and world.get_compile_error().is_empty())

	# Let it run for four seconds with its controller script.
	for i in 240:
		await physics_frame
	check("no simulation error", not world.has_simulation_error())
	check_near("simulated in step with the game", world.get_time(), 4.0, 0.05)

	# The script drives each joint towards a moving target; they should be close to it.
	var time: float = world.get_time()
	var yaw: Node3D = world.get_node("Turret/Yaw")
	check_near("the arm's yaw follows its target", yaw.joint_position, 0.9 * sin(0.5 * time), 0.1)
	var shoulder: Node3D = world.get_node("Turret/UpperArm/Shoulder")
	check_near("the shoulder follows its target", shoulder.joint_position, 0.3 * sin(0.8 * time), 0.1)
	var finger: Node3D = world.get_node("Turret/UpperArm/Forearm/Wrist/Hand/FingerLeft/Slide")
	check("the gripper stays within its travel", finger.joint_position > -0.005 and finger.joint_position < 0.05)

	# The blocks have come to rest on the sprung platform, which has sunk under them.
	var platform: Node3D = world.get_node("Platform")
	check("the platform has sunk on its spring", platform.position.y < 0.29 and platform.position.y > 0.15)
	for block_name in ["Block1", "Block2", "Block3"]:
		var block: Node3D = world.get_node(block_name)
		check("%s rests on the platform" % block_name, absf(block.position.y - (platform.position.y + 0.08)) < 0.02)
	demo.free()


func test_shapes() -> void:
	var scene: PackedScene = load("res://demos/shapes.tscn")
	check("demo scene loads", scene != null)
	var demo: Node3D = scene.instantiate()
	root.add_child(demo)
	var world: Node3D = demo.get_node("MujocoWorld")
	check("demo compiles", world.is_compiled() and world.get_compile_error().is_empty())

	world.paused = true
	world.step(20000)  # 40 seconds
	for shape_name in ["Sphere", "Box", "Capsule", "Cylinder"]:
		var body: Node3D = world.get_node(shape_name)
		check("%s has landed" % shape_name, body.position.y > 0.2 and body.position.y < 1.0)
	# The pendulum's limits (-120 to 30 degrees) leave room for it to hang straight down.
	var pendulum: Node3D = world.get_node("Pendulum")
	check_near("the pendulum hangs straight down", pendulum.rotation.z, -PI / 2, 0.05)
	check("no simulation error", not world.has_simulation_error())
