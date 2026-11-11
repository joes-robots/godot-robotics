extends Node3D
## Phase 3 demo. MuJoCo's humanoid (models/humanoid.xml, from the MuJoCo repository,
## Apache-2.0) was converted into models/humanoid.tscn with the FileSystem dock's
## "Convert to MuJoCo Scene", and is instanced here. Its motors are off, so it falls
## like a rag doll. It starts over every few seconds with a random nudge, or when R
## is pressed.
##
## Its two hamstring tendons (MujocoTendon nodes under the world) couple each hip
## and knee. The import leaves out only the file's cameras, lights, keyframes and
## textures, none of which change how it moves.

## Simulated seconds before starting over.
@export var restart_after := 4.0
## Largest horizontal speed of the nudge, in m/s.
@export var nudge := 1.5

@onready var world: MujocoWorld = $Humanoid
@onready var torso: MujocoBody = $Humanoid/torso


func _ready() -> void:
	restart.call_deferred()


func _physics_process(_delta: float) -> void:
	if world.get_simulation_time() >= restart_after:
		restart()


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed and not event.echo and event.keycode == KEY_R:
		restart()


func restart() -> void:
	world.reset()
	torso.set_linear_velocity(Vector3(randf_range(-nudge, nudge), 0, randf_range(-nudge, nudge)))
