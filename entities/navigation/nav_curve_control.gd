
@tool
extends Node3D

enum ControlType {
	IN,
	OUT
}


@export var func_godot_properties: Dictionary = {}
@export var point_index : int = 1
@export var target : String
@export var control_type := ControlType.IN


func _func_godot_apply_properties(props: Dictionary) -> void:
	if props.has("point_index") :
		point_index = props["point_index"]
	if props.has("control_type"):
		control_type = props["control_type"]
	if props.has("target"):
		target = props["target"]


func _func_godot_build_complete() -> void:
	var target_node : Node = get_tree().get_first_node_in_group(target)
	if target_node is Path3D:
		var path := target_node as Path3D
		
		var point_global_position : Vector3 = path.curve.get_point_position(point_index) + path.global_position
		if control_type == ControlType.IN:
			path.curve.set_point_in(point_index, global_position - point_global_position)
		else:
			path.curve.set_point_out(point_index, global_position - point_global_position)
		
		queue_free()  # After contol point is added, this node is not needed
