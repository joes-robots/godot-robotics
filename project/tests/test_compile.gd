extends "res://tests/test_base.gd"
## When MujocoWorld compiles: batching, failures, and freeing with a compile queued.

var compiles := 0


func _on_compiled() -> void:
	compiles += 1


func run() -> void:
	var world := make_world("World")
	world.model_compiled.connect(_on_compiled)
	var body: MujocoBody = add(world, MujocoBody.new(), "Body")
	var geom: MujocoGeom = add(body, MujocoGeom.new(), "Geom")
	await process_frame
	compiles = 0

	# Many edits in one frame cause a single compile at the end of the frame.
	for i in 10:
		geom.radius = 0.1 + i * 0.01
		geom.size = Vector3(1, 1 + i * 0.1, 1)
	check("no compile until the frame ends", compiles == 0)
	await process_frame
	check("10 edits in one frame -> 1 compile (got %d)" % compiles, compiles == 1)

	compiles = 0
	world.name = "Renamed"
	body.position = Vector3(0, 1, 0)
	add(body, MujocoGeom.new(), "Extra")
	await process_frame
	check("rename + move + add in one frame -> 1 compile (got %d)" % compiles, compiles == 1)
	check("model name follows the node name", world.get_mjcf().contains("model=\"Renamed\""))

	# A failed compile keeps the last good model. A body with a joint but no geom has no mass,
	# which MuJoCo rejects.
	compiles = 0
	var massless: MujocoBody = add(world, MujocoBody.new(), "Massless")
	add(massless, MujocoJoint.new(), "Hinge")
	print("(the next compile error is expected)")
	await process_frame
	check("failed compile does not emit model_compiled", compiles == 0)
	var xml := world.get_mjcf()
	check("last good model kept", xml.contains("name=\"Body\"") and not xml.contains("name=\"Massless\""))

	add(massless, MujocoGeom.new(), "Geom")
	await process_frame
	check("fixing the error compiles again", compiles == 1 and world.get_mjcf().contains("name=\"Massless\""))

	# Freeing the world with a compile queued must not crash (the queued call is dropped).
	geom.radius = 0.5
	world.free()
	await process_frame
	await process_frame
	check("freed with a compile queued", true)
