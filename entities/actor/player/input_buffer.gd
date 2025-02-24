
extends Node
class_name InputBuffer




#==================================================================================================
# --------------
# | PROPERTIES |
# -------------- 

@export var action_name   : String = ""
@export var buffer_window : int = 1




#==================================================================================================
# -------------
# | VARIABLES |
# -------------

var frames_left : int = 0




#==================================================================================================
# -----------
# | METHODS |
# -----------


func is_input_just_pressed() -> bool:
	if frames_left:
		frames_left = 0
		return true
		
	return false




func _process(_delta : float) -> void:
	frames_left = maxi(0, frames_left - 1)
	
	if Input.is_action_just_pressed(action_name):
		frames_left = buffer_window
