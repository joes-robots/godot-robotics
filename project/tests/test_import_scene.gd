extends "res://tests/test_base.gd"
## What the editor's MJCF import commands are built on: an imported world remembers
## its source file, verify() works before the world is in the tree, and
## import_to_scene() saves a scene with its meshes next to it.

const OUT_DIR := "user://test_import_scene"


func remove_recursive(path: String) -> void:
	for dir in DirAccess.get_directories_at(path):
		remove_recursive(path.path_join(dir))
	for file in DirAccess.get_files_at(path):
		DirAccess.remove_absolute(path.path_join(file))
	DirAccess.remove_absolute(path)


func run() -> void:
	var importer := MujocoImporter.new()
	var world := importer.import_file("res://tests/models/arm.xml")
	check("the world remembers its source", world.source_file == "res://tests/models/arm.xml")
	var diffs := importer.verify(world, 200)
	check("verify compiles a world outside the tree %s" % [diffs], diffs.is_empty())
	world.free()

	# A file outside the project keeps its absolute path.
	var outside := OS.get_temp_dir().path_join("godot_robotics_test_model.xml")
	var file := FileAccess.open(outside, FileAccess.WRITE)
	file.store_string("<mujoco><worldbody><geom size='0.1'/></worldbody></mujoco>")
	file.close()
	world = importer.import_file(outside)
	check("absolute source outside the project (%s)" % world.source_file, world.source_file == outside)
	world.free()
	DirAccess.remove_absolute(outside)

	# import_to_scene: the scene, its meshes, and the check.
	DirAccess.make_dir_recursive_absolute(OUT_DIR)
	var scene_path := OUT_DIR.path_join("meshes.tscn")
	check("import_to_scene succeeds", importer.import_to_scene("res://tests/models/mesh.xml", scene_path) == OK)
	check("verified %s" % [importer.get_verify_result()], importer.get_verify_result().is_empty())
	var mesh_dir := MujocoImporter.mesh_directory_for_scene(scene_path)
	check("meshes next to the scene (%s)" % mesh_dir, mesh_dir == OUT_DIR.path_join("meshes_meshes"))
	check("mesh files saved", FileAccess.file_exists(mesh_dir.path_join("tetra.res")) and FileAccess.file_exists(mesh_dir.path_join("wedge.res")))
	var scene: PackedScene = load(scene_path)
	check("the scene loads", scene != null)
	if scene:
		var loaded: MujocoWorld = scene.instantiate()
		check("with its source", loaded.source_file == "res://tests/models/mesh.xml")
		var geom: MujocoGeom = loaded.get_node("tetra_body/tetra_a")
		check("and its geoms use the mesh files", geom.mesh != null and geom.mesh.resource_path == mesh_dir.path_join("tetra.res"))
		root.add_child(loaded)
		await process_frame
		check("and it simulates", loaded.get_mjcf().contains("tetra"))
		loaded.queue_free()
		await process_frame

	# A model without meshes makes no mesh folder.
	var arm_path := OUT_DIR.path_join("arm.tscn")
	check("arm imports", importer.import_to_scene("res://tests/models/arm.xml", arm_path, 0) == OK)
	check("no mesh folder without meshes", not DirAccess.dir_exists_absolute(MujocoImporter.mesh_directory_for_scene(arm_path)))

	# A file that doesn't load saves nothing and says why.
	print("(the next error is expected)")
	var bad_path := OUT_DIR.path_join("missing.tscn")
	check("a missing source fails", importer.import_to_scene("res://tests/models/missing.xml", bad_path) != OK)
	check("nothing saved", not FileAccess.file_exists(bad_path))
	check("the report says why", importer.get_report().size() > 0 and importer.get_report()[0].contains("Could not load"))

	remove_recursive(OUT_DIR)
