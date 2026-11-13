# Architecture

Design decisions for the MuJoCo extension, in the order they were made. Each entry says what was chosen, what the alternative was, and why. Decisions marked *provisional* are due to be revisited at the milestone named.

## Fixed scope

- MuJoCo simulates only the MuJoCo node tree. Godot's own physics is left alone; there is no `PhysicsServer3D` backend.
- GDExtension built with godot-cpp `10.0.0-stable`, `api_version` 4.7. MuJoCo 3.10.0, pinned in `MUJOCO_VERSION`.

## Milestone 1: world node and options

| # | Decision | Chosen | Alternative | Why |
|---|----------|--------|-------------|-----|
| 1 | Name of the root node | `MujocoWorld` | `MujocoSpec` | The node owns the spec, the compiled model and the data, and will do the stepping. |
| 2 | Lifetime of the `mjSpec` | *Superseded by milestone 2's decision 1.* Originally long-lived: one spec for the node's life, edited in place, recompiled with `mj_recompile`. | Rebuild a fresh spec from the node tree on every compile. | Keeps state across edits. The cost is that child nodes must keep their spec elements in sync; that cost only appears once there are child nodes. |
| 3 | Where option values live | In the node's own fields, copied into the spec at compile time. | Directly in `spec->option`. | Works with either answer to decision 2. |
| 4 | How options are declared | One table of (name, type, default, field), served through `_get`, `_set` and `_get_property_list`. | One getter and setter per option. | Far less code, and the same pattern is needed later for elements with default classes. |
| 5 | When an option change takes effect | Written straight into the live `mjModel.opt`; no recompile. | Recompile on every change, or once per frame. | Options don't change the model's structure. |
| 6 | A failed compile | Keeps the last good model and data, prints the error, and exposes `is_compiled()` and `get_compile_error()`. | Remove the node from the tree. | A bad value in the editor must not make a node disappear. |
| 7 | An engine error (from `mj_step` and friends) | Deferred to milestone 2, when stepping exists. Plan: the log handler sets a flag, the world sees it after stepping, stops, and resets its data. | | Nothing steps yet. |
| 8 | Vector options (gravity, wind, magnetic) | Shown in Godot's convention and converted when written to MuJoCo, through one conversion header. | Store MuJoCo's values as they are. | The inspector should read like the rest of Godot. |

### How decision 6 is implemented

`MujocoWorld::compile()` builds the new model separately and only replaces the old one after a success, so a failed compile leaves the last good model untouched.

### How decisions 3, 4, 5 and 8 are implemented

- `MujocoWorld` holds one `mjOption options` field, in MuJoCo's own layout and conventions. That is the stored value of every option (decision 3).
- `src/mujoco_options.cpp` has the table: one row per option with its name, kind (real, int, enum, flags, vector) and byte offset inside `mjOption`. `MujocoWorld` forwards `_get`, `_set`, `_get_property_list`, `_property_can_revert` and `_property_get_revert` to it (decision 4). Defaults are read from `mj_defaultOption`, not written in the table.
- `compile()` copies `options` into `spec->option`. `_set` also copies it into `model->opt` when a model exists, so no recompile is needed (decision 5).
- Vector options are converted in the table's get and set, using `src/mujoco_convert.h` (decision 8). Because the stored value is MuJoCo's double-precision one, an option that is never touched keeps MuJoCo's exact default even though Godot's `Vector3` is single precision.

## Milestone 2: bodies and geoms

| # | Decision | Chosen | Alternative | Why |
|---|----------|--------|-------------|-----|
| 1 | Source of truth and spec lifetime (replaces milestone 1's decision 2) | The node tree is the source of truth. Every compile builds a fresh `mjSpec` by walking the tree; simulation state resets. | A long-lived spec kept in sync incrementally; or rebuild plus carrying state over by name. | Nothing to keep in sync, so editor operations (undo, duplicate, reparent) can't leave the spec stale. Carrying state over can be added later without changing this. |
| 2 | Element base class | `MujocoElement` (extends `Node3D`) shared by body, geom and later types. | Independent classes. | One place for finding the world, the MuJoCo name and recompile requests. |
| 3 | How elements get into the spec | Each element has a virtual `add_to_spec()`; the world calls it while walking the tree. | One function in the world with a branch per type. | A new element type touches only its own file. |
| 4 | Names in MuJoCo | The node's path relative to the world, e.g. `Arm/Hand`. | The node's own name; or a separate property. | Always unique, and maps a node straight to its MuJoCo id. To be revisited for import. |
| 5 | Transforms | A body's transform is its pose relative to its parent body. Plain `Node3D` nodes in between are allowed and folded in. MuJoCo's origin is the world node's origin. Scale is ignored with a warning. | | Matches MJCF, and allows grouping nodes. |
| 6 | Geoms | One `MujocoGeom` class with a shape enum; sizes in MuJoCo's own terms (half-extents, radius, half-length). | A class per shape; Godot-style full sizes. | Values match MJCF files and MuJoCo's documentation exactly. |
| 7 | Drawing geoms | A geom draws itself in the editor and the game, with a switch to turn that off. | Invisible geoms with user-supplied meshes. | Useful out of the box; the switch covers custom visuals. |
| 8 | Stepping and pose sync | In the world's `_physics_process`, only while the game runs, with an accumulator and a step cap. The world writes body transforms after stepping. `paused` property and `step()` method. | Each body reading its own pose. | One pass in one place. |
| 9 | Making something move | A minimal `MujocoJoint` (free type only) in this milestone; milestone 3 completes it. | A `free` checkbox on the body. | Mirrors MJCF's `<freejoint/>`. |
| 10 | Attribute storage | Table-driven from the start, recording which attributes were explicitly set. | Plain members now, retrofit for default classes later. | The retrofit touched every element in the first pass. |
| 11 | When structural changes recompile | Once per frame: changes set a flag and the world compiles at the end of the frame. | Immediately on each change. | Many changes in a frame cost one compile. |
| 12 | Engine errors | The log handler sets a flag; after stepping the world stops, reports once, and resets on the next reset or recompile. | | Stepping arrives in this milestone. |

### How decisions 1 to 5 and 11 are implemented

- `MujocoWorld::compile()` makes a new spec, calls `add_children_to_spec()` on itself, compiles, and on success replaces the spec, model and data together. On failure the previous three stay.
- `add_children_to_spec()` walks the tree depth first, carrying the current MuJoCo body and the transform accumulated since that body. A `MujocoElement` adds itself through `add_to_spec()` and returns the body its children belong to (or nullptr for "don't look at my children"). Any other node is looked through. A nested `MujocoWorld` is skipped: it is its own model.
- `MujocoElement` asks for a recompile when it enters or leaves the tree, is renamed or moved, or its children are reordered. `MujocoWorld::request_recompile()` queues one deferred `compile()` per frame.
- `MujocoWorld::to_mjcf()` writes the last compiled spec as MJCF. The tests read the model through it.

### How decisions 6 and 10 are implemented

- **Override storage (decision 10).** `MujocoElement` holds one `overrides` dictionary: attribute name to value, for the attributes explicitly set on that element. It is the only thing saved to scenes. Reading an attribute returns its override, or else its inherited value. Assigning null removes the override, which is also how the inspector's revert arrow works. "Set to the default's value" and "not set" are therefore different, as in MJCF.
- **Per element class:** a static table of `MujocoAttribute` rows (name, type, inspector hint), plus a `read` and a `write` function that move one attribute between a Godot value and the MuJoCo spec struct (`mjsGeom`). Inherited values come from `read` on a struct filled in by MuJoCo (`mjs_defaultGeom`), so they can't drift from the library. `add_to_spec` calls `write` for overridden attributes only.
- **Geom size (decision 6).** Half-sizes as in MuJoCo, laid out along Godot's axes: a box's y and z are swapped relative to MJCF. *The single `size` vector was later replaced by named fields; see milestone 3, "Naming and layout of attributes".*
- **Mass.** MuJoCo marks "no mass given" with NaN; the `mass` attribute shows that as -1.
- **Precision.** `Vector3` and `Color` are single precision in standard Godot builds, so sizes, friction and colours carry about 7 significant digits. Scalar attributes (density, mass) are doubles.

### How decisions 8, 9 and 12 are implemented

- **Stepping.** `MujocoWorld::_physics_process` adds the frame's duration to an accumulator and takes as many whole MuJoCo timesteps as fit, carrying the remainder over. At most 0.1 s is simulated per frame; beyond that the backlog is dropped. Nothing steps in the editor, while `paused`, before a model exists, or after an engine error. `step(n)` advances by hand regardless of `paused`; `reset()` returns to the initial state; `get_time()` is simulated time.
- **Pose sync.** After a successful compile the world looks up each `MujocoBody` node's id in the model by name and keeps the list. After stepping it sets each body node's global transform from MuJoCo's `xpos` and `xquat`, relative to the world node. Nodes are held by instance id, so a freed node is skipped.
- **Rest pose.** The model is built from where bodies were placed, not from where the simulation has moved them. A `MujocoBody` remembers its transform from before the simulation first moved it, and `get_spec_transform()` gives the world that. Moving a body yourself sets a new rest pose and recompiles. A move made by the simulation is flagged so it doesn't count as an edit.
- **Recompiling during play** therefore returns every body to its rest pose at time zero, as decision 1 says.
- **Joints.** `MujocoJoint` has one attribute, `type` (free, ball, slide, hinge; MuJoCo's default is hinge). Its node position is the joint's anchor.
- **Engine errors.** The log handler sets an atomic flag when MuJoCo reports an error. The world clears it before stepping and checks it after each step; if set, it stops stepping and says so once. `reset()` or a successful compile clears the stop.
- **Queued and direct compiles.** `request_recompile()` queues `compile_if_pending()`, which does nothing if `compile()` was called directly in the meantime.

### How decision 7 is implemented

- A `MujocoGeom` in the tree owns one internal `MeshInstance3D` child holding a Godot primitive mesh built from its shape and size (full sizes, so twice MuJoCo's half-sizes) and a material in its `rgba`. It is rebuilt whenever an attribute changes, through `MujocoElement::attributes_changed()`.
- Being internal, the child is not saved, not shown in the scene tree, and not seen by the world's tree walk.
- `draw_shape` (a plain property, default on) removes it. It is separate from `Node3D.visible`, which would also hide a geom's own children.
- An infinite plane is drawn 200 m across.
- **Selecting in the editor.** The editor tests clicks against gizmos, and only nodes belonging to the scene get gizmos, so the internal mesh can't be clicked. `MujocoGeomGizmoPlugin` (editor only, in `src/mujoco_editor_plugin.cpp`) gives each geom a gizmo whose collision triangles are its shape.
- **A geom with no `MujocoBody` above it** belongs to MuJoCo's world body: fixed scenery, as with a `<geom>` directly inside `<worldbody>` in MJCF.

## Milestone 3: joints and the runtime API

| # | Decision | Chosen | Alternative | Why |
|---|----------|--------|-------------|-----|
| 1 | Joint axis | An `axis` attribute in the joint node's own frame, so rotating the node rotates the axis. | Attribute only, ignoring node rotation; or node orientation only. | Reads like MJCF when the node is unrotated, and the rotate gizmo still works. |
| 2 | Angle units | Radians in scripts and saved scenes; the inspector shows degrees. The spec is compiled with `compiler.degree = 0`. | Degrees everywhere. | MuJoCo's runtime state is always radians. |
| 3 | Joint attributes | `type`, `axis`, `limited`, `range_lower`, `range_upper`, `ref`, `springref`, `stiffness`, `damping`, `armature`, `frictionloss`. Limits are a checkbox plus two numbers, greyed out until it is ticked. *Revised after trying it in the editor: a `range` of 0 to 0 read as "locked" when it meant "no limit".* | One `range` vector with no checkbox, limited when set. | Says what it does, and has no x/y labels. |
| 4 | Where state access lives | Methods on each node for single elements, plus bulk arrays on the world (`qpos`, `qvel`). Each element learns its model id after every compile. | World-only access by name. | Convenient per element, complete for whole-state use. |
| 5 | Joint state types | `joint_position` and `joint_velocity` are floats, for hinge and slide joints. (Not `position`: that is the node's own, the joint's anchor.) They show in the inspector, greyed out for ball and free joints. | One property whose type depends on the joint type. | The one-number case is nearly all scripted use. |
| 6 | Moving a body during play | Only a mocap body's move goes into the running simulation. Moving any other body, free-jointed ones included, places it: new rest pose, recompile. | Also teleporting free-jointed bodies without a recompile. | One rule: to drive a body from outside, make it mocap. |
| 7 | Mocap bodies | A `mocap` attribute on `MujocoBody`. | | MJCF's own mechanism for bodies driven from outside. |
| 8 | Applying forces | MuJoCo's semantics: `applied_force` and `applied_torque` persist until changed. | Godot-style per-frame `apply_force`. | Maps one-to-one onto `xfrc_applied` and `qfrc_applied`. |
| 9 | Attribute changes during play | Some attributes are "live" and write straight into the compiled model: joint `damping`, `stiffness`, `frictionloss`, `armature`, `range`; geom `friction`, `rgba`. Others recompile. | Every change recompiles. | Tuning while watching the result. |
| 10 | Signals | `compiled` and `stepped` on the world. | | Scripts that cache ids or sizes need the first. |
| 11 | Joint gizmo | Drawn in the editor at the anchor along the axis, clickable, and large enough not to be hidden inside geoms. | | Joints are otherwise invisible. |
| 12 | Class icons | One per node class. | | The classes are otherwise indistinguishable from `Node3D` in the scene tree. |

### Standing rule: attributes that don't apply are greyed out

An attribute that means nothing in the element's current state (a free joint's axis) stays in the inspector, greyed out (`PROPERTY_USAGE_READ_ONLY`). Its override is kept and is not written to the model, so switching back restores it. An element class implements this with `customize_attribute_property()`; an attribute whose value changes the outcome reports true from `changes_attribute_presentation()` so the inspector is rebuilt.

### Naming and layout of attributes (revised after trying it in the editor)

- **One meaning per field.** MJCF packs several meanings into one attribute (`size`, `friction`, `range`). In the inspector each meaning is its own named attribute: `radius`, `half_length`, `half_extents`, `plane_half_size`, `grid_spacing`; `friction_sliding`, `friction_torsional`, `friction_rolling`; `range_lower`, `range_upper`. `read` and `write` map them onto MuJoCo's packed fields.
- **Readable names where MJCF's are cryptic:** `color` for `rgba`.
- **Hidden or greyed out.** Attributes that are alternatives to one another (a geom's size fields) are hidden when they don't belong to the current shape. Attributes that are merely switched off (a free joint's axis, a range while `limited` is off) are greyed out. Either way the stored value is kept and not written to the model.
- **Geom size defaults** are ours, not MuJoCo's zero: radius 0.5, half-length 0.5, half-extents 0.5, an infinite plane with grid spacing 1. A new geom of any shape compiles as it is.
- **`limited` with an empty range** (lower not below upper) leaves the joint unlimited instead of failing the compile, since that is the state just after ticking the box.
- **A default joint is a hinge about the vertical axis** (MuJoCo's default). Gravity doesn't turn it; that is not a limit.

### Joint gizmo (decision 11)

`MujocoJointGizmoPlugin` draws each joint in the editor in its own frame: a hinge as an axis line with an arrowhead and a ring, a slide as a double-headed line, a ball as three rings, a free joint as three short lines. It is drawn on top of everything, and its length is 1.5 times the farthest reach of the geoms on the same body (at least 0.25 m), so geometry never hides it. The lines are also what a click is tested against.

### How decisions 4, 5 and 10 are implemented

- **Ids.** Each element class says what kind of MuJoCo object it is (`get_mujoco_type()`). After a successful compile the world looks every element up by name and gives it its id (`set_mujoco_id()`); -1 means "not in the model". An element that leaves the world drops its id. Scripts read it with `get_mujoco_id()`.
- **Access to the simulation.** `MujocoWorld::get_model()` and `get_data()` hand element classes the compiled model and its state. After writing state directly, a class calls `state_changed()`, which runs `mj_forward` and moves the body nodes.
- **Joint state.** `joint_position` and `joint_velocity` read and write the joint's entry in `qpos` and `qvel`. They are ordinary properties shown in the inspector and not saved; `_validate_property` greys them out on ball and free joints and in the editor, and shows a hinge's as degrees. Setting either takes effect at once.
- **Whole state.** `MujocoWorld.get_qpos()`, `set_qpos()`, `get_qvel()` and `set_qvel()` copy the arrays as they are, in MuJoCo's layout and coordinate convention. `MujocoJoint.get_qpos_address()` and `get_qvel_address()` say where a joint's entries start.
- **Signals.** `compiled` after each successful compile; `stepped` after the simulation has advanced and the nodes have moved, once per physics frame or per `step()` call.

### How decisions 6 to 8 are implemented

- **Mocap.** `mocap` is a `MujocoBody` attribute written to the spec. Whether a body in the running simulation is mocap is read from the compiled model (`body_mocapid`), not from the attribute.
- **Being moved.** `MujocoElement` calls the virtual `transform_changed_by_user()` when its node is moved by anything but the simulation; the default asks for a recompile. `MujocoBody` overrides it: a mocap body in a running game writes its pose, relative to the world node, into `mocap_pos` and `mocap_quat` and calls `state_changed()`; anything else sets a new rest pose and recompiles. A mocap body keeps its rest pose, so `reset()` puts it back.
- **Forces.** `MujocoBody.applied_force` and `applied_torque` are the body's six entries in `xfrc_applied`; `MujocoJoint.applied_force` is the joint's entry in `qfrc_applied`. They are state: shown in the inspector, not saved, greyed out in the editor, cleared by `reset()` and by recompiling.
- **Velocities.** `MujocoBody.get_linear_velocity()` and `get_angular_velocity()` use `mj_objectVelocity`.
- **Frames.** Forces, torques and velocities are in Godot's global coordinates. The simulation's frame is the world node's, so they are rotated by the world node's rotation on the way in and out.
- **Up to date after a step.** `mj_step` computes poses and velocities and then advances the state, leaving them one timestep behind. After each batch of steps the world calls `mj_kinematics`, `mj_comPos` and `mj_comVel`, so nodes and velocities match the state just reached. Contacts and sensor values are not refreshed this way and still describe the previous timestep.

### Class icons (decision 12)

`project/bin/icons/` holds one 16 by 16 SVG per node class, named after the class, in the colour Godot uses for 3D nodes. The `[icons]` section of `mujoco_physics.gdextension` maps each class to its file.

### How decision 9 is implemented

- When an attribute changes, `MujocoElement::apply_attribute_change()` decides how it takes effect. In a running game, for an element that is in the compiled model, it offers the change to the class's `write_live()`. If that returns true the value has been written into the model and nothing else happens; otherwise, and always in the editor, it asks for a recompile.
- Live for joints: `damping`, `frictionloss`, `armature`, `stiffness`, `limited`, `range_lower`, `range_upper`. Live for geoms: the three friction values and `color`. Everything else rebuilds.
- `armature` is inertia, and MuJoCo derives constants from the model's inertia. `MujocoWorld::refresh_model_constants()` recomputes them with `mj_setConst`, setting the joint positions aside and restoring them, because that function puts the model into its reference pose.
- The override is stored in the node as usual, so the next rebuild produces the same model.
- `to_mjcf()` writes the spec of the last compile, so it does not show live changes made since.

### How decisions 1 to 3 are implemented

- `MujocoJoint::add_to_spec` writes overridden attributes that apply to the joint's type. The axis is then rotated by the node's rotation in its body and converted; it is left at MuJoCo's default when the node is unrotated and no axis is set.
- `range_lower`, `range_upper`, `ref` and `springref` get a degrees display hint on hinge and ball joints and none on slides, through `customize_attribute_property()`.
- `stiffness` and `damping` are entry 0 of MuJoCo's polynomial coefficient arrays, i.e. the ordinary constant values.

## Conventions

- Coordinates: Godot (x, y, z) is MuJoCo (x, -z, y). Godot is y-up, MuJoCo is z-up.
- Quaternions: the vector part converts like a vector; Godot orders components (x, y, z, w), MuJoCo (w, x, y, z).
- All conversions live in `src/mujoco_convert.h`.
