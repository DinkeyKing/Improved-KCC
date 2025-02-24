
@tool
extends DirectionalLight3D




#==================================================================================================
# --------------
# | PROPERTIES |
# --------------


@export var func_godot_properties : Dictionary = {}




#==================================================================================================
# -----------
# | METHODS |
# -----------


func _ready() -> void :
	shadow_enabled = true




func _func_godot_apply_properties(props : Dictionary) -> void :
	
	if props.has("color") :
		light_color = props["color"]
	if props.has("energy") :
		light_energy = props["energy"]
