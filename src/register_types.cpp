#include "register_types.h"

#include <gdextension_interface.h>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/classes/editor_plugin_registration.hpp>
#include <godot_cpp/godot.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include "mujoco_body.h"
#include "mujoco_editor_plugin.h"
#include "mujoco_element.h"
#include "mujoco_geom.h"
#include "mujoco_joint.h"
#include "mujoco_world.h"
#include <mujoco/mujoco.h>

using namespace godot;

static mjfLogHandler previous_mujoco_log_handler = nullptr;

// Route MuJoCo log messages into Godot's output panel. MuJoCo's default handler
// calls exit() on errors, which would kill the editor. Compile errors never reach
// here (the compiler installs its own handler and reports via mjs_getError); this
// catches engine errors, e.g. during mj_step. MuJoCo expects error handlers not to
// return, so after an error the affected mjData must be treated as invalid.
static void mujoco_log_handler(const mjLogMessage *p_msg)
{
	String text = String("MuJoCo: ") + String::utf8(p_msg->subject);
	if (p_msg->body) {
		text += "\n" + String::utf8(p_msg->body);
	}
	const char *func = p_msg->func ? p_msg->func : "mujoco";
	const char *file = p_msg->file ? p_msg->file : "mujoco";

	switch (p_msg->level) {
		case mjLOG_ERROR:
			_err_print_error(func, file, p_msg->line, text, true, false);
			break;
		case mjLOG_WARNING:
			_err_print_error(func, file, p_msg->line, text, false, true);
			break;
		default:
			UtilityFunctions::print_verbose(text);
			break;
	}
}

void initialize_gdextension_types(ModuleInitializationLevel p_level)
{
	if (p_level == MODULE_INITIALIZATION_LEVEL_EDITOR) {
		// Only reached in editor builds of Godot (never in exported games). The plugin
		// itself is only instantiated when the editor UI is running.
		GDREGISTER_INTERNAL_CLASS(MujocoGeomGizmoPlugin);
		GDREGISTER_INTERNAL_CLASS(MujocoJointGizmoPlugin);
		GDREGISTER_INTERNAL_CLASS(MujocoEditorPlugin);
		EditorPlugins::add_by_type<MujocoEditorPlugin>();
		return;
	}
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
	previous_mujoco_log_handler = mju_setLogHandler(mujoco_log_handler);
	GDREGISTER_CLASS(MujocoWorld);
	GDREGISTER_ABSTRACT_CLASS(MujocoElement);
	GDREGISTER_CLASS(MujocoBody);
	GDREGISTER_CLASS(MujocoGeom);
	GDREGISTER_CLASS(MujocoJoint);
}

void uninitialize_gdextension_types(ModuleInitializationLevel p_level) {
	if (p_level == MODULE_INITIALIZATION_LEVEL_EDITOR) {
		EditorPlugins::remove_by_type<MujocoEditorPlugin>();
		return;
	}
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
	// Restore MuJoCo's handler so it never calls into an unloaded library.
	mju_setLogHandler(previous_mujoco_log_handler);
	previous_mujoco_log_handler = nullptr;
}

extern "C"
{
	// Initialization
	GDExtensionBool GDE_EXPORT mujoco_physics_library_init(GDExtensionInterfaceGetProcAddress p_get_proc_address, GDExtensionClassLibraryPtr p_library, GDExtensionInitialization *r_initialization)
	{
		GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);
		init_obj.register_initializer(initialize_gdextension_types);
		init_obj.register_terminator(uninitialize_gdextension_types);
		init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);

		return init_obj.init();
	}
}