#pragma once

#include <godot_cpp/variant/array.hpp>
#include <mujoco/mujoco.h>

// Compares two compiled MuJoCo models and lists where they differ.
//
// Used to check an import: p_original is the model compiled from the MJCF
// file, p_other the one compiled from the imported node tree.
//
// Every numeric array in the model is compared, using MuJoCo's own list of its
// model's fields, so a field can't be forgotten. Then both models are
// simulated for p_steps timesteps from their initial state and the resulting
// joint positions are compared.
//
// Numbers are equal if they agree to about six significant digits: Godot
// stores node transforms and vectors in single precision, so an imported
// model can't match more closely than that.
//
// Not compared: names of elements the original left unnamed (ours always have
// one); the bounding-volume trees MuJoCo builds for collision detection, which
// are derived from the geoms and reorder themselves at the slightest
// difference; and memory sizes.
//
// Returns one Dictionary per differing field:
//   field           the mjModel field, e.g. "geom_size"
//   message         what differs, in words
//   count           how many entries differ
//   max_difference  the largest difference found (0 for counts and names)
godot::Array mujoco_compare_models(const mjModel *p_original, const mjModel *p_other, int p_steps);
