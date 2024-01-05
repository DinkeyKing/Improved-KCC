extends Node3D
class_name PlayerCamera

##############
# PROPERTIES #
##############

@export_group("General")
@export var sensitivity : float = 0.005
@export var max_rotation_up : float = 90.0
@export var max_rotation_down : float = -90.0

@export_group("Height effect")
@export var stand_height : float = 0.5

#########
# NODES #
#########

@onready var player : Player = self.get_parent() as Player
@onready var pivot_x : Node3D = self.get_child(0) as Node3D
@onready var pivot_z : Node3D = self.get_child(0).get_child(0) as Node3D

#############
# CONSTANTS #
#############

const Y_MOTION_LERP_SPEED : float = 40.0 # Default: 40.0

#############
# VARIABLES #
#############

var height_offset : float = 0.0

#############
# FUNCTIONS #
#############

func _ready() -> void:
	height_offset = stand_height

func _unhandled_input(event : InputEvent) -> void:
	if Input.mouse_mode != Input.MOUSE_MODE_CAPTURED :
		return
	if event is InputEventMouseMotion :
		rotate_y(-event.relative.x * sensitivity)
		pivot_x.rotate_x(-event.relative.y * sensitivity)
		pivot_x.rotation.x = clamp(pivot_x.rotation.x, deg_to_rad(max_rotation_down), deg_to_rad(max_rotation_up))
		
func _process(delta : float) -> void:
	global_position.x = player.global_position.x
	global_position.z = player.global_position.z
	
	var height_goal : float = player.global_position.y + height_offset
	global_position.y = lerpf(global_position.y, height_goal, Y_MOTION_LERP_SPEED * delta)
