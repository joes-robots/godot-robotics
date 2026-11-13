extends "res://tests/test_base.gd"
## MujocoBody: bodies in the node tree become bodies in the compiled model,
## with names from node paths and poses converted to MuJoCo's frame.


func run() -> void:
	check("MujocoBody is registered", ClassDB.class_exists("MujocoBody"))
	check("MujocoBody is a MujocoElement", ClassDB.is_parent_class("MujocoBody", "MujocoElement"))
	check("MujocoElement can't be created on its own", not ClassDB.can_instantiate("MujocoElement"))

	var world: Node3D = add(root, ClassDB.instantiate("MujocoWorld"), "World")

	# A body under the world. Godot (1, 2, 3) is MuJoCo (1, -3, 2).
	var arm: Node3D = ClassDB.instantiate("MujocoBody")
	arm.position = Vector3(1, 2, 3)
	add(world, arm, "Arm")
	check("name is the path from the world", arm.get_mujoco_name() == "Arm")

	# A child body: its pose is relative to its parent. A quarter turn about
	# Godot's up axis (y) is a quarter turn about MuJoCo's up axis (z).
	var hand: Node3D = ClassDB.instantiate("MujocoBody")
	hand.position = Vector3(0, 1, 0)
	hand.rotation = Vector3(0, PI / 2, 0)
	add(arm, hand, "Hand")
	check("nested name is the path from the world", hand.get_mujoco_name() == "Arm/Hand")

	# A plain Node3D between the world and a body is folded into the body's pose.
	var group: Node3D = add(world, Node3D.new(), "Group")
	group.position = Vector3(5, 0, 0)
	var crate: Node3D = ClassDB.instantiate("MujocoBody")
	crate.position = Vector3(0, 0, -1)
	add(group, crate, "Crate")

	check("compiles", world.compile())
	var bodies := mjcf_elements(world.to_mjcf())
	check("three bodies in the model", bodies.has("Arm") and bodies.has("Arm/Hand") and bodies.has("Group/Crate"))
	check_numbers("body position is converted", bodies["Arm"].get("pos", ""), "1 -3 2")
	check_numbers("child position is relative to its parent", bodies["Arm/Hand"].get("pos", ""), "0 0 1")
	check_numbers("child rotation is converted (w x y z)", bodies["Arm/Hand"].get("quat", ""), "0.70711 0 0 0.70711")
	check_numbers("grouping node is folded in", bodies["Group/Crate"].get("pos", ""), "5 1 0")
	check("the child body is nested in MJCF", world.to_mjcf().find("name=\"Arm/Hand\"") > world.to_mjcf().find("name=\"Arm\""))

	# Changes recompile by themselves, once, at the end of the frame.
	arm.position = Vector3(0, 4, 0)
	check_numbers("not recompiled yet in the same frame", mjcf_elements(world.to_mjcf())["Arm"].get("pos", ""), "1 -3 2")
	await process_frame
	check_numbers("moving a body recompiles", mjcf_elements(world.to_mjcf())["Arm"].get("pos", ""), "0 0 4")

	hand.name = "Gripper"
	await process_frame
	bodies = mjcf_elements(world.to_mjcf())
	check("renaming a body recompiles", bodies.has("Arm/Gripper") and not bodies.has("Arm/Hand"))

	group.remove_child(crate)
	await process_frame
	check("removing a body recompiles", not mjcf_elements(world.to_mjcf()).has("Group/Crate"))
	check("a body outside a world has no MuJoCo name", crate.get_mujoco_name() == "")

	arm.add_child(crate)
	await process_frame
	check("adding a body recompiles", mjcf_elements(world.to_mjcf()).has("Arm/Crate"))

	# Scale is ignored (with a warning): MuJoCo has none.
	group.scale = Vector3(2, 2, 2)
	var pin: Node3D = ClassDB.instantiate("MujocoBody")
	pin.position = Vector3(1, 0, 0)
	add(group, pin, "Pin")
	check("compiles with a scaled node", world.compile())
	check_numbers("scale doesn't move children", mjcf_elements(world.to_mjcf())["Group/Pin"].get("pos", ""), "6 0 0")

	# A world inside a world is a separate model.
	var inner: Node3D = add(world, ClassDB.instantiate("MujocoWorld"), "Inner")
	add(inner, ClassDB.instantiate("MujocoBody"), "Part")
	world.compile()
	inner.compile()
	check("outer model leaves out the inner world's bodies", not world.to_mjcf().contains("Part"))
	check("inner model has its own body", mjcf_elements(inner.to_mjcf()).has("Part"))
