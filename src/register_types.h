#ifndef MUJOCO_PHYSICS_REGISTER_TYPES_H
#define MUJOCO_PHYSICS_REGISTER_TYPES_H

#include <godot_cpp/godot.hpp>

void initialize_gdextension_types(godot::ModuleInitializationLevel p_level);
void uninitialize_gdextension_types(godot::ModuleInitializationLevel p_level);

// True if MuJoCo has reported an error since this was last called. Asking
// clears it. MuJoCo expects never to continue after an error, so whatever was
// being simulated when one happened must be treated as invalid.
bool mujoco_take_engine_error();

#endif // MUJOCO_PHYSICS_REGISTER_TYPES_H