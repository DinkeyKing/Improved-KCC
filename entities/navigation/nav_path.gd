
@tool
extends Path3D



const BOX_MESH_LENGTH : float = 1.0
const PADDING_LENGTH  : float = 0.5


@export var func_godot_properties: Dictionary = {}

var sphere_mesh: Mesh = preload("res://entities/navigation/path_sphere_mesh.tres")
var box_mesh: Mesh = preload("res://entities/navigation/path_box_mesh.tres")
var mesh_rids: Array[RID]



func _func_godot_apply_properties(props: Dictionary) -> void:
	# Create new curve with an origin point
	curve = Curve3D.new()
	curve.add_point(Vector3.ZERO)
	
	# Set targetname
	if props.has("targetname"):
		GameManager.set_targetname(self, props["targetname"])
	
	if not Engine.is_editor_hint(): return
	
	var map := GameManager.get_edited_map_node(self)
	if map:
		map.build_complete.connect(_on_build_complete)




func _on_build_complete() -> void:
	build_path_mesh()




func _ready() -> void:
	if Engine.is_editor_hint(): return
	
	build_path_mesh()




func free_meshes() -> void:
	for rid in mesh_rids:
		if rid.is_valid(): RenderingServer.free_rid(rid)
	mesh_rids.clear()




func _exit_tree() -> void:
	free_meshes()




func build_path_mesh() -> void:
	free_meshes()
	
	
	var xform : Transform3D
	var scenario: RID = get_world_3d().scenario
	
	
	var path_begin_mesh_instance: RID = RenderingServer.instance_create()
	mesh_rids.append(path_begin_mesh_instance)
	RenderingServer.instance_set_scenario(path_begin_mesh_instance, scenario)
	RenderingServer.instance_set_base(path_begin_mesh_instance, sphere_mesh)
	
	xform = Transform3D(Basis(), global_position)
	RenderingServer.instance_set_transform(path_begin_mesh_instance, xform)
	
	
	var path_end_mesh_instance: RID = RenderingServer.instance_create()
	mesh_rids.append(path_end_mesh_instance)
	RenderingServer.instance_set_scenario(path_end_mesh_instance, scenario)
	RenderingServer.instance_set_base(path_end_mesh_instance, sphere_mesh)
	
	var end_point_global_position: Vector3 = curve.get_point_position(curve.point_count - 1) + global_position
	xform = Transform3D(Basis(), end_point_global_position)
	RenderingServer.instance_set_transform(path_end_mesh_instance, xform)
	
	
	var curve_length: float = curve.get_baked_length()
	var division_count: int = int(curve_length / (BOX_MESH_LENGTH + PADDING_LENGTH * 2.0))
	var division_length: float = curve_length / (division_count + 1)
	
	for i in division_count:
		var offset: float = (i + 1) * division_length
		xform = curve.sample_baked_with_rotation(offset, false, true)
		xform.origin += global_position
		
		var path_box_mesh_instance: RID = RenderingServer.instance_create()
		mesh_rids.append(path_box_mesh_instance)
		RenderingServer.instance_set_scenario(path_box_mesh_instance, scenario)
		RenderingServer.instance_set_base(path_box_mesh_instance, box_mesh)
		
		#xform = Transform3D(Basis.looking_at(tangent), pos)
		RenderingServer.instance_set_transform(path_box_mesh_instance, xform)
		
		
