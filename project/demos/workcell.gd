extends Node3D
## Drives the robot arm in the workcell demo.
##
## There are no actuators yet, so this script does a motor's job by hand: every
## physics frame it pushes each joint towards a target with a force proportional
## to how far off it is (a spring, in effect). The joints' own `damping`
## attribute, set in the scene, keeps that from oscillating.
##
## It uses three things from the runtime API: `joint_position` to read a joint,
## `applied_force` to push it, and the world's `get_time()` as the clock.

## One entry per driven joint: where it is in the scene, how hard to push per
## unit of error, and its target as a centre, a swing and a speed.
const JOINTS := [
	{"path": "Turret/Yaw", "gain": 150.0, "centre": 0.0, "swing": 0.9, "speed": 0.5, "phase": 0.0},
	{"path": "Turret/UpperArm/Shoulder", "gain": 500.0, "centre": 0.0, "swing": 0.3, "speed": 0.8, "phase": 0.0},
	{"path": "Turret/UpperArm/Forearm/Elbow", "gain": 300.0, "centre": 0.0, "swing": 0.45, "speed": 0.8, "phase": 1.2},
	{"path": "Turret/UpperArm/Forearm/Wrist/Roll", "gain": 20.0, "centre": 0.0, "swing": 1.2, "speed": 1.1, "phase": 0.0},
	{"path": "Turret/UpperArm/Forearm/Wrist/Hand/Pitch", "gain": 20.0, "centre": 0.0, "swing": 0.5, "speed": 1.6, "phase": 0.5},
]

## The two fingers open and close together.
const FINGERS := ["Turret/UpperArm/Forearm/Wrist/Hand/FingerLeft/Slide", "Turret/UpperArm/Forearm/Wrist/Hand/FingerRight/Slide"]
const FINGER_GAIN := 200.0
const FINGER_TRAVEL := 0.03

@onready var world: MujocoWorld = $MujocoWorld


func _physics_process(_delta: float) -> void:
	if not world.is_compiled():
		return
	var time := world.get_time()

	for entry in JOINTS:
		var joint: MujocoJoint = world.get_node(entry.path)
		var target: float = entry.centre + entry.swing * sin(entry.speed * time + entry.phase)
		joint.applied_force = entry.gain * (target - joint.joint_position)

	# 0 is closed (as built); the fingers open to FINGER_TRAVEL and back.
	var opening := FINGER_TRAVEL * (0.5 + 0.5 * sin(1.3 * time))
	for path in FINGERS:
		var finger: MujocoJoint = world.get_node(path)
		finger.applied_force = FINGER_GAIN * (opening - finger.joint_position)
