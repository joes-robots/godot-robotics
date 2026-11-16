extends SceneTree
## Imports every MJCF file under a folder and checks each against its original.
## Run it through scripts/survey_mjcf.sh.
##
## For each file it prints one line:
##   MATCH   the imported model has no differences from the original
##   DIFFER  it imported, but the compiled models differ (in how many fields)
##   FAILED  MuJoCo could not load the file, or the import does not compile
## With --verbose after the folder, the report and differences are printed too.


func find_xml(directory: String, found: Array) -> void:
	var dir := DirAccess.open(directory)
	if dir == null:
		return
	for entry in dir.get_directories():
		find_xml(directory.path_join(entry), found)
	for entry in dir.get_files():
		if entry.get_extension() == "xml":
			found.append(directory.path_join(entry))


func _initialize() -> void:
	await process_frame
	var arguments := OS.get_cmdline_user_args()
	if arguments.is_empty():
		print("usage: mjcf_survey.gd -- <folder> [--verbose]")
		quit(2)
		return
	var verbose := arguments.has("--verbose")

	var files := []
	find_xml(arguments[0], files)
	files.sort()

	var matched := 0
	var differed := 0
	var failed := 0
	for path in files:
		var importer: Object = ClassDB.instantiate("MujocoImporter")
		var world: Node3D = importer.import_file(path)
		var label: String = path.trim_prefix(arguments[0]).trim_prefix("/")
		if world == null:
			failed += 1
			print("FAILED  %s" % label)
			if verbose:
				print("        ", importer.get_error().strip_edges().replace("\n", " "))
			continue

		root.add_child(world)
		world.paused = true
		if not world.is_compiled():
			failed += 1
			print("FAILED  %s (the imported tree does not compile)" % label)
			if verbose:
				print("        ", world.get_compile_error().strip_edges().replace("\n", " "))
		else:
			var differences: Array = importer.verify_file(world, path)
			if differences.is_empty():
				matched += 1
				print("MATCH   %s" % label)
			else:
				differed += 1
				print("DIFFER  %s (%d fields)" % [label, differences.size()])
			if verbose:
				for line in importer.get_report_text().strip_edges().split("\n", false):
					print("        report: ", line)
				for entry in differences:
					print("        differs: %s: %s" % [entry.field, entry.message])
		root.remove_child(world)
		world.free()

	print("\n%d files: %d match, %d differ, %d failed." % [files.size(), matched, differed, failed])
	quit(0)
