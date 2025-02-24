@tool
extends Node3D
class_name PlayerCamera


#==================================================================================================
# --------------
# | PROPERTIES |
# --------------

@export_group("General")
@export var base_mouse_sensitivity : float = 0.005
@export var max_rotation_up : float = 90.0
@export var max_rotation_down : float = -90.0
@export var stand_height : float = 0.5
@export var y_pos_lerp_speed : float = 40.0




#==================================================================================================
# ---------
# | NODES |
# ---------

@onready var player  := self.get_parent() as Node3D
@onready var pivot_x := self.get_child(0) as Node3D
@onready var pivot_z := self.get_child(0).get_child(0) as Node3D
@onready var camera  := self.get_child(0).get_child(0).get_child(0) as Camera3D




#==================================================================================================
# -------------
# | CONSTANTS |
# -------------

const MAX_Y_DIFF : float = 0.26           # Default: 0.26 (This value should be about the same as the max player step height)



#==================================================================================================
# -------------
# | VARIABLES |
# -------------

var height_offset : float = 0.0




#==================================================================================================
# -----------
# | METHODS |
# -----------


func _ready() -> void:
	if Engine.is_editor_hint(): return
	
	SETTINGS.settings_changed.connect(_on_settings_changed)
	
	height_offset = stand_height
	camera.fov = SETTINGS.camera_fov
	top_level = true




func _on_settings_changed(setting: GameSettings.Setting) -> void:
	if setting == GameSettings.Setting.CAMERA_FOV:
		camera.fov = SETTINGS.camera_fov





func _unhandled_input(event : InputEvent) -> void:
	if Engine.is_editor_hint(): return
	
	if Input.mouse_mode != Input.MOUSE_MODE_CAPTURED :
		return
	if event is InputEventMouseMotion :
		var mouse_motion := event as InputEventMouseMotion
		
		var sensitivity: float = base_mouse_sensitivity * SETTINGS.mouse_sensivity
		rotate_y(-mouse_motion.relative.x * sensitivity)
		pivot_x.rotate_x(-mouse_motion.relative.y * sensitivity)
		
		pivot_x.rotation.x = clampf(pivot_x.rotation.x, deg_to_rad(max_rotation_down), deg_to_rad(max_rotation_up))




func _process(delta : float) -> void:
	if Engine.is_editor_hint(): return
	
	# The x and z position is the same as the player's.
	global_position.x = player.global_position.x
	global_position.z = player.global_position.z
	
	var y_pos_goal : float = player.global_position.y + height_offset
	
	if not is_zero_approx(y_pos_lerp_speed):
		# The y position is interpolated to avoid teleportation during snapping and stair climbing.
		global_position.y = lerpf(global_position.y, y_pos_goal, y_pos_lerp_speed * delta)
		
		# Limit how far the camera can go from it's normal position
		var y_diff : float = player.global_position.y + height_offset - global_position.y  # Positive means below
		if absf(y_diff) >  MAX_Y_DIFF :
			global_position.y += y_diff - (MAX_Y_DIFF * signf(y_diff) )
	else :
		global_position.y = y_pos_goal




func apply_rotation_degrees(angles: Vector3) -> void:
	rotation_degrees = Vector3(0.0, angles.y, 0.0)
	pivot_x.rotation_degrees = Vector3(angles.x, 0.0, 0.0)
	pivot_z.rotation_degrees = Vector3(0.0, 0.0,angles.z)
