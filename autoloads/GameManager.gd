extends Node
class_name GameManager

#############
# FUNCTIONS #
#############

func _ready() -> void:
	Input.mouse_mode = Input.MOUSE_MODE_CAPTURED

func _process(_delta : float) -> void:
	if Input.is_action_just_pressed("pause"):
		if Input.mouse_mode == Input.MOUSE_MODE_CAPTURED:
			Input.mouse_mode = Input.MOUSE_MODE_VISIBLE
		else :
			Input.mouse_mode = Input.MOUSE_MODE_CAPTURED


func get_body_by_rid(rid : RID) -> Object :
	var body_id : int = PhysicsServer3D.body_get_object_instance_id(rid)
	var body : Object = instance_from_id(body_id)
	
	return body
