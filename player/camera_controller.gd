extends Node3D
class_name PlayerCamera



# --------------
# | PROPERTIES |
# --------------

@export_group("General")
@export var sensitivity : float = 0.005
@export var max_rotation_up : float = 90.0
@export var max_rotation_down : float = -90.0

@export_group("Height effect")
@export var stand_height : float = 0.5



# ---------
# | NODES |
# ---------

@onready var player  := self.get_parent() as Node3D
@onready var pivot_x := self.get_child(0) as Node3D
@onready var pivot_z := self.get_child(0).get_child(0) as Node3D
@onready var camera  := self.get_child(0).get_child(0).get_child(0) as Camera3D



# -------------
# | CONSTANTS |
# -------------

const Y_MOTION_LERP_SPEED : float = 40.0  # Default: 40.0
const MAX_Y_DIFF : float = 0.26           # Default: 0.26 (This value should be about the same as the max player step height)



# -------------
# | VARIABLES |
# -------------

var height_offset : float = 0.0



# -------------
# | FUNCTIONS |
# -------------

func _ready() -> void:
	height_offset = stand_height


func _unhandled_input(event : InputEvent) -> void:
	if Input.mouse_mode != Input.MOUSE_MODE_CAPTURED :
		return
	if event is InputEventMouseMotion :
		var mouse_motion := event as InputEventMouseMotion
		
		rotate_y(-mouse_motion.relative.x * sensitivity)
		pivot_x.rotate_x(-mouse_motion.relative.y * sensitivity)
		
		pivot_x.rotation.x = clampf(pivot_x.rotation.x, deg_to_rad(max_rotation_down), deg_to_rad(max_rotation_up))


func _process(delta : float) -> void:
	# The x and z position is the same as the player's.
	global_position.x = player.global_position.x
	global_position.z = player.global_position.z
	
	# The y position is interpolated to avoid teleportation during snapping and stair climbing.
	var y_pos_goal : float = player.global_position.y + height_offset
	global_position.y = lerpf(global_position.y, y_pos_goal, Y_MOTION_LERP_SPEED * delta)
	
	# Limit how far the camera can go from it's normal position
	var y_diff : float = player.global_position.y + height_offset - global_position.y  # Positive means below
	if absf(y_diff) >  MAX_Y_DIFF :
		global_position.y += y_diff - (MAX_Y_DIFF * signf(y_diff) )
