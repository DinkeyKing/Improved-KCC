
@tool
extends Node3D
class_name CurvePoint


@export var func_godot_properties: Dictionary = {}
@export var index : int = 1
@export var tilt : float = 0.0
@export var in_pos := Vector3.ZERO
@export var out_pos := Vector3.ZERO
@export var target : String



func _func_godot_apply_properties(props: Dictionary) -> void:
	if props.has("index") :
		index = props["index"]
	if props.has("tilt") :
		tilt = props["tilt"]
	if props.has("in_pos") :
		in_pos = props["in_pos"]
	if props.has("out_pos") :
		out_pos = props["out_pos"]
	if props.has("target"):
		target = props["target"]



func _func_godot_build_complete() -> void:
	var target_node : Node = get_tree().get_first_node_in_group(target)
	if target_node is Path3D:
		var path := target_node as Path3D
		var point_position : Vector3 = global_position - path.global_position
		
		path.curve.add_point(point_position, in_pos, out_pos, index)
		path.curve.set_point_tilt(index, tilt)
		queue_free()  # After point is added, this node is not needed
