extends Node3D
## Phase 2 demo. A two-link arm follows a moving target: inverse kinematics turns the
## target into joint angles, which become the targets of the arm's position servos.
## A mocap stirrer, moved by this script, pushes balls around a pen. Sensors feed the
## readout above the arm.
##
## Everything is driven by simulation time, so the demo runs the same at any frame rate.

## Target circle, relative to the shoulder in the arm's plane (x right, y up), in meters.
@export var target_center := Vector2(1.1, 0.4)
@export var target_radius := 0.45
## Angular speed of the target around its circle, in rad/s.
@export var target_speed := 0.6

## Stirrer circle (in the world's frame) and angular speed.
@export var stir_center := Vector3(-3, 0.35, 0)
@export var stir_radius := 1.0
@export var stir_speed := 1.2

## Link lengths: joint to joint, and elbow to tip.
const UPPER_LENGTH := 1.0
const LOWER_LENGTH := 1.0

@onready var world: MujocoWorld = $MujocoWorld
@onready var upper: MujocoBody = $MujocoWorld/Base/Upper
@onready var shoulder_servo: MujocoActuator = $MujocoWorld/Base/Upper/Shoulder/Servo
@onready var elbow_servo: MujocoActuator = $MujocoWorld/Base/Upper/Lower/Elbow/Servo
@onready var tip_position: MujocoSensor = $MujocoWorld/Base/Upper/Lower/Tip/Position
@onready var shoulder_torque: MujocoSensor = $MujocoWorld/Base/Upper/Shoulder/Servo/Torque
@onready var stirrer: MujocoBody = $MujocoWorld/Stirrer
@onready var target: Node3D = $Target
@onready var readout: Label3D = $Readout

## The shoulder hinge sits at the upper link's origin, which never moves.
var shoulder_origin: Vector3


func _ready() -> void:
	shoulder_origin = upper.global_position


func _physics_process(_delta: float) -> void:
	var t := world.get_simulation_time()

	var planar := target_center + target_radius * Vector2(cos(target_speed * t), sin(target_speed * t))
	target.global_position = shoulder_origin + Vector3(planar.x, planar.y, 0)
	var angles := solve_ik(planar)
	shoulder_servo.ctrl = angles.x
	elbow_servo.ctrl = angles.y

	# A mocap body goes wherever its node is put.
	stirrer.position = stir_center + stir_radius * Vector3(cos(stir_speed * t), 0, sin(stir_speed * t))


func _process(_delta: float) -> void:
	var error: float = (tip_position.get_value() as Vector3).distance_to(target.global_position)
	readout.text = "tip error  %.3f m\nshoulder torque  %.1f N m" % [error, shoulder_torque.get_value()]


## Joint angles (shoulder, elbow) that put the tip at p_target, in the arm's plane
## relative to the shoulder. Angles are from the scene pose (both links along +X),
## positive counterclockwise (the hinges' axes point along +Z, toward the camera).
## Of the two solutions this picks the one with the elbow raised.
func solve_ik(p_target: Vector2) -> Vector2:
	var reach := p_target.length()
	var cos_elbow := clampf((reach * reach - UPPER_LENGTH * UPPER_LENGTH - LOWER_LENGTH * LOWER_LENGTH)
			/ (2.0 * UPPER_LENGTH * LOWER_LENGTH), -1.0, 1.0)
	var elbow := -acos(cos_elbow)
	var shoulder := atan2(p_target.y, p_target.x) - atan2(LOWER_LENGTH * sin(elbow), UPPER_LENGTH + LOWER_LENGTH * cos(elbow))
	return Vector2(shoulder, elbow)
