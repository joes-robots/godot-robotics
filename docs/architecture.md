# Architecture

Design decisions for the MuJoCo extension, in the order they were made. Each entry says what was chosen, what the alternative was, and why. Decisions marked *provisional* are due to be revisited at the milestone named.

## Fixed scope

- MuJoCo simulates only the MuJoCo node tree. Godot's own physics is left alone; there is no `PhysicsServer3D` backend.
- GDExtension built with godot-cpp `10.0.0-stable`, `api_version` 4.7. MuJoCo 3.10.0, pinned in `MUJOCO_VERSION`.

## Milestone 1: world node and options

| # | Decision | Chosen | Alternative | Why |
|---|----------|--------|-------------|-----|
| 1 | Name of the root node | `MujocoWorld` | `MujocoSpec` | The node owns the spec, the compiled model and the data, and will do the stepping. |
| 2 | Lifetime of the `mjSpec` | Long-lived: one spec for the node's life, edited in place, recompiled with `mj_recompile` so simulation state survives edits. *Provisional: revisit in milestone 2.* | Rebuild a fresh spec from the node tree on every compile. | Keeps state across edits. The cost is that child nodes must keep their spec elements in sync; that cost only appears once there are child nodes. |
| 3 | Where option values live | In the node's own fields, copied into the spec at compile time. | Directly in `spec->option`. | Works with either answer to decision 2. |
| 4 | How options are declared | One table of (name, type, default, field), served through `_get`, `_set` and `_get_property_list`. | One getter and setter per option. | Far less code, and the same pattern is needed later for elements with default classes. |
| 5 | When an option change takes effect | Written straight into the live `mjModel.opt`; no recompile. | Recompile on every change, or once per frame. | Options don't change the model's structure. |
| 6 | A failed compile | Keeps the last good model and data, prints the error, and exposes `is_compiled()` and `get_compile_error()`. | Remove the node from the tree. | A bad value in the editor must not make a node disappear. |
| 7 | An engine error (from `mj_step` and friends) | Deferred to milestone 2, when stepping exists. Plan: the log handler sets a flag, the world sees it after stepping, stops, and resets its data. | | Nothing steps yet. |
| 8 | Vector options (gravity, wind, magnetic) | Shown in Godot's convention and converted when written to MuJoCo, through one conversion header. | Store MuJoCo's values as they are. | The inspector should read like the rest of Godot. |

### How decision 6 is implemented

`mj_recompile` deletes the model and data it is given when it fails. To keep the last good pair, `MujocoWorld::compile()` recompiles *copies* (`mj_copyModel`, `mj_copyData`) and swaps them in only after a success. This costs one model copy per recompile.

### How decisions 3, 4, 5 and 8 are implemented

- `MujocoWorld` holds one `mjOption options` field, in MuJoCo's own layout and conventions. That is the stored value of every option (decision 3).
- `src/mujoco_options.cpp` has the table: one row per option with its name, kind (real, int, enum, flags, vector) and byte offset inside `mjOption`. `MujocoWorld` forwards `_get`, `_set`, `_get_property_list`, `_property_can_revert` and `_property_get_revert` to it (decision 4). Defaults are read from `mj_defaultOption`, not written in the table.
- `compile()` copies `options` into `spec->option`. `_set` also copies it into `model->opt` when a model exists, so no recompile is needed (decision 5).
- Vector options are converted in the table's get and set, using `src/mujoco_convert.h` (decision 8). Because the stored value is MuJoCo's double-precision one, an option that is never touched keeps MuJoCo's exact default even though Godot's `Vector3` is single precision.

## Conventions

- Coordinates: Godot (x, y, z) is MuJoCo (x, -z, y). Godot is y-up, MuJoCo is z-up.
