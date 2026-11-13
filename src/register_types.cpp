#include "register_types.h"

#include <gdextension_interface.h>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <mujoco/mujoco.h>

#include "mujoco_world.h"

using namespace godot;

static mjfLogHandler prev_mujoco_log_handler = nullptr;

// Mujoco exits on error - catch and print these as Godot's errors.
// At this point the Mujoco model/data are no longer valid.
static void mujoco_log_handler(const mjLogMessage* p_msg) {
	String mujoco_msg = "MuJoCo: ";

	mujoco_msg += String::utf8(p_msg->subject) + "\n";
	if (p_msg->body) {
		mujoco_msg += "    " + String::utf8(p_msg->body) + "\n";
	}

	const char* func = p_msg->func ? p_msg->func : "no function";
	const char* file = p_msg->file ? p_msg->file : "no file";

	switch(p_msg->level) {
		case mjLOG_ERROR:
			_err_print_error(func, file, p_msg->line, mujoco_msg, true, false);
			break;
		case mjLOG_WARNING:
			_err_print_error(func, file, p_msg->line, mujoco_msg, false, true);
			break;
		default:
			UtilityFunctions::print_verbose(mujoco_msg);
			break;
	}
	// TODO: make sure we are not using old/invalid model/data after this point
}

void initialize_gdextension_types(ModuleInitializationLevel p_level)
{
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
	prev_mujoco_log_handler = mju_setLogHandler(mujoco_log_handler);
	GDREGISTER_CLASS(MujocoWorld);
}

void uninitialize_gdextension_types(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
	mju_setLogHandler(prev_mujoco_log_handler);
	prev_mujoco_log_handler = nullptr;
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